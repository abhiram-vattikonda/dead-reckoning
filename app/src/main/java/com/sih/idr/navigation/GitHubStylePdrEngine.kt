package com.sih.idr.navigation

import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState
import com.sih.idr.navigation.pdr.ComplementaryHeadingFilter
import com.sih.idr.navigation.pdr.DynamicStepCounter
import com.sih.idr.navigation.pdr.FixedStrideLength
import com.sih.idr.navigation.pdr.GyroscopeBiasCalibrator
import com.sih.idr.navigation.pdr.GyroscopeIntegrator
import com.sih.idr.navigation.pdr.GyroscopeOrientationDcm
import com.sih.idr.navigation.pdr.MagnetometerHeadingProvider
import com.sih.idr.navigation.pdr.PdrConfig
import com.sih.idr.navigation.pdr.PdrDebug
import com.sih.idr.navigation.pdr.StrideLengthProvider
import com.sih.idr.utils.GeoUtils
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * PEDESTRIAN-STYLE BASELINE ported from nisargnp/DeadReckoning — NOT the final
 * vehicle/INS solution, and NOT claimed to meet any SIH accuracy target. Its job:
 * (1) a working inertial-localization reference, (2) something for MMM/vehicle
 * algorithms to beat, (3) verification of the sensor pipeline.
 *
 * Algorithm per IMU sample:
 *  1. linear accel → magnitude a = sqrt(lx²+ly²+lz²) (OS sensor if present,
 *     else raw − gravity estimate)
 *  2. [DynamicStepCounter]: moving-average hysteresis peak detector → step?
 *  3. gyro − bias → high-pass (0.0025) → ×dt → DCM update → gyro heading,
 *     plus magnetic initial heading ([MagnetometerHeadingProvider])
 *  4. [ComplementaryHeadingFilter]: h = 0.02·mag + 0.98·gyro (wrap-aware)
 *  5. on step: dx = stride·cos(h); dy = stride·sin(h); x += dx; y += dy
 *
 * LOCAL FRAME (documented once): x = meters East, y = meters North of the
 * [initialize] GPS seed; heading h is the math angle from +x (East). Converted to
 * lat/lon per step via a local tangent-plane projection ([GeoUtils.moveEnu]) —
 * never by accumulating lat/lon with magic constants.
 *
 * GPS ISOLATION: [onGnssMeasurement] is inherited no-op — after [initialize] the
 * engine consumes IMU only. Implements [DeadReckoningEngine], so swapping in
 * MMMDeadReckoningEngine later touches one construction site in the ViewModel.
 */
class GitHubStylePdrEngine(
    val config: PdrConfig = PdrConfig(),
    private val strideProvider: StrideLengthProvider =
        FixedStrideLength(config.strideLengthMeters)
) : DeadReckoningEngine, EngineDebugReporter {

    private val stepCounter = DynamicStepCounter(config.stepSensitivity)
    private val biasCal = GyroscopeBiasCalibrator(config.gyroBiasTrials)
    private val integrator = GyroscopeIntegrator(
        thresholdRps = config.gyroThresholdRps,
        biasX = config.gyroBiasX, biasY = config.gyroBiasY, biasZ = config.gyroBiasZ
    )
    private val dcm = GyroscopeOrientationDcm()
    private val headingFilter = ComplementaryHeadingFilter(
        magWeight = config.magWeight, gyroWeight = config.gyroWeight
    )

    private var state: NavigationState? = null
    override val currentState: NavigationState? get() = state

    // Local tangent plane (meters from seed).
    private var xEast = 0.0
    private var yNorth = 0.0
    private var originLat = 0.0
    private var originLon = 0.0
    private var originAlt = 0.0

    // Heading state (radians, math frame).
    private var initialHeading = 0f
    private var initialHeadingLatched = false
    private var lastMagHeading = Float.NaN
    private var lastGyroHeading = 0f
    private var lastFusedHeading = 0f

    // Gravity fallback: TYPE_GRAVITY → quaternion-derived → low-pass raw accel.
    private var lpGravX = Float.NaN
    private var lpGravY = Float.NaN
    private var lpGravZ = Float.NaN

    private var elapsedS = 0.0
    private var lastSampleTimeNanos = -1L
    private var lastStepTimeNanos = -1L
    private var lastStepVelE = 0f
    private var lastStepVelN = 0f

    /** Latest intermediates for logging/CSV (read-only snapshot). */
    override var lastDebug: PdrDebug? = null
        private set

    override fun initialize(initialState: NavigationState) {
        reset()
        state = initialState
        originLat = initialState.latitude
        originLon = initialState.longitude
        originAlt = initialState.altitude
        if (!config.autoCalibrateGyroBias) {
            biasCal.setManualBias(config.gyroBiasX, config.gyroBiasY, config.gyroBiasZ)
        }
    }

    override fun processImu(sample: ImuSample): NavigationState {
        val prev = state ?: return NavigationState(
            timestampNanos = sample.timestampNanos, latitude = 0.0, longitude = 0.0
        )

        if (lastSampleTimeNanos > 0) {
            val dt = (sample.timestampNanos - lastSampleTimeNanos) / 1e9
            if (dt > 0.0 && dt < 5.0) elapsedS += dt
        }
        lastSampleTimeNanos = sample.timestampNanos

        // ---- 1. linear accel magnitude ----
        val (lx, ly, lz) = linearAccel(sample)
        val accMag = sqrt((lx * lx + ly * ly + lz * lz).toDouble()).toFloat()
        val step = stepCounter.findStep(accMag.toDouble())
        val stride = strideProvider.strideLengthMeters()

        // ---- 2/3. gyro bias → delta angles → DCM heading ----
        if (config.autoCalibrateGyroBias && !biasCal.isCalibrated) {
            biasCal.addSample(sample.gyroX, sample.gyroY, sample.gyroZ)
            integrator.setBias(biasCal.biasX(), biasCal.biasY(), biasCal.biasZ())
        }
        val delta = integrator.update(
            sample.timestampNanos, sample.gyroX, sample.gyroY, sample.gyroZ
        )
        val dcmHeading = dcm.update(delta[0], delta[1], delta[2])

        // ---- magnetic heading (needs gravity + mag) ----
        val grav = gravityVector(sample)
        val mx = sample.magnetometerX
        val my = sample.magnetometerY
        val mz = sample.magnetometerZ
        if (grav != null && mx != null && my != null && mz != null) {
            val mh = MagnetometerHeadingProvider.heading(
                grav[0], grav[1], grav[2], mx, my, mz,
                config.magBiasX, config.magBiasY, config.magBiasZ,
                Math.toRadians(config.declinationDeg)
            )
            if (!mh.isNaN()) {
                lastMagHeading = mh
                if (!initialHeadingLatched) {
                    // Repo behavior: gyro frame anchored to first magnetic heading.
                    initialHeading = mh
                    initialHeadingLatched = true
                }
            }
        }
        lastGyroHeading = dcmHeading + initialHeading

        // ---- 4. complementary fusion (gyro carries it if mag is stale/absent) ----
        lastFusedHeading = if (lastMagHeading.isNaN()) lastGyroHeading
        else headingFilter.fuse(lastMagHeading, lastGyroHeading)

        // ---- 5. step → position ----
        if (step) {
            val dx = stride * cos(lastFusedHeading)
            val dy = stride * sin(lastFusedHeading)
            xEast += dx
            yNorth += dy
            // Velocity ≈ step displacement / time since previous step.
            val prevStep = lastStepTimeNanos
            lastStepTimeNanos = sample.timestampNanos
            val dtStep = if (prevStep < 0) 0.6
            else ((sample.timestampNanos - prevStep) / 1e9).coerceIn(0.2, 5.0)
            lastStepVelE = (dx / dtStep).toFloat()
            lastStepVelN = (dy / dtStep).toFloat()
        }

        val (lat, lon) = GeoUtils.moveEnu(originLat, originLon, yNorth, xEast)

        // Step-velocity decays to zero after 2 s without steps (standing still reads 0).
        val sinceStepS = if (lastStepTimeNanos < 0) Double.MAX_VALUE
        else (sample.timestampNanos - lastStepTimeNanos) / 1e9
        val still = sinceStepS > 2.0
        val velE = if (still) 0f else lastStepVelE
        val velN = if (still) 0f else lastStepVelN

        val next = prev.copy(
            timestampNanos = sample.timestampNanos,
            latitude = lat,
            longitude = lon,
            altitude = originAlt,
            velEast = velE,
            velNorth = velN,
            headingDeg = MagnetometerHeadingProvider.toCompassDeg(lastFusedHeading),
            confidenceMeters = (2.0 + 0.5 * elapsedS).toFloat()
        )
        state = next

        lastDebug = PdrDebug(
            timestampNanos = sample.timestampNanos,
            accMagnitude = accMag,
            stepDetected = step,
            strideLength = stride,
            gyroHeadingRad = lastGyroHeading,
            magHeadingRad = lastMagHeading,
            fusedHeadingRad = lastFusedHeading,
            xEast = xEast.toFloat(),
            yNorth = yNorth.toFloat(),
            latitude = lat,
            longitude = lon,
            forwardSpeedMps = Float.NaN,
            yawRateDegS = Float.NaN,
            lateralVelMps = Float.NaN,
            snapMode = "PDR"
        )
        return next
    }

    /** Linear accel: OS sensor → else raw minus gravity estimate. */
    private fun linearAccel(s: ImuSample): Triple<Float, Float, Float> {
        if (s.linearX != null && s.linearY != null && s.linearZ != null) {
            return Triple(s.linearX, s.linearY, s.linearZ)
        }
        val g = gravityVector(s)
        return if (g != null) Triple(s.accelX - g[0], s.accelY - g[1], s.accelZ - g[2])
        else Triple(s.accelX, s.accelY, s.accelZ)
    }

    /**
     * Gravity fallback chain: TYPE_GRAVITY → derived from rotation-vector
     * quaternion → slow low-pass of raw accel (adapts when held steady).
     */
    private fun gravityVector(s: ImuSample): FloatArray? {
        if (s.gravityX != null && s.gravityY != null && s.gravityZ != null) {
            return floatArrayOf(s.gravityX, s.gravityY, s.gravityZ)
        }
        if (s.quatX != null && s.quatY != null && s.quatZ != null && s.quatW != null) {
            val m = com.sih.idr.navigation.CoordinateTransformer
                .rotationMatrixFromQuaternion(s.quatX, s.quatY, s.quatZ, s.quatW)
            val out = FloatArray(3)
            com.sih.idr.navigation.CoordinateTransformer.gravityInPhoneFrame(m, out)
            return out
        }
        // Last resort: first-order low-pass of raw accel.
        if (lpGravX.isNaN()) {
            lpGravX = s.accelX; lpGravY = s.accelY; lpGravZ = s.accelZ
        } else {
            val a = 0.05f
            lpGravX += a * (s.accelX - lpGravX)
            lpGravY += a * (s.accelY - lpGravY)
            lpGravZ += a * (s.accelZ - lpGravZ)
        }
        return floatArrayOf(lpGravX, lpGravY, lpGravZ)
    }

    override fun reset() {
        state = null
        stepCounter.reset()
        biasCal.reset()
        integrator.reset()
        integrator.setBias(config.gyroBiasX, config.gyroBiasY, config.gyroBiasZ)
        dcm.reset()
        xEast = 0.0; yNorth = 0.0
        originLat = 0.0; originLon = 0.0; originAlt = 0.0
        initialHeading = 0f; initialHeadingLatched = false
        lastMagHeading = Float.NaN; lastGyroHeading = 0f; lastFusedHeading = 0f
        lpGravX = Float.NaN; lpGravY = Float.NaN; lpGravZ = Float.NaN
        elapsedS = 0.0
        lastSampleTimeNanos = -1L; lastStepTimeNanos = -1L
        lastStepVelE = 0f; lastStepVelN = 0f
        lastDebug = null
    }
}
