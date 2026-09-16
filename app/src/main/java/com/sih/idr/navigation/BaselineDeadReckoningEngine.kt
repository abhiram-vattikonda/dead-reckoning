package com.sih.idr.navigation

import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState
import com.sih.idr.utils.GeoUtils
import kotlin.math.atan2

/**
 * Deliberately SIMPLE strapdown baseline. Honest drift is expected — this is the
 * reference that the future MMM/AI engine must beat (<10% drift target is NOT
 * claimed here; consumer-phone double integration cannot do that).
 *
 * Pipeline per IMU sample:
 *  1. dt from sensor timestamps (clamped to [0, 0.2]s; gaps larger than that reset dt).
 *  2. Orientation: latest rotation-vector quaternion -> phone-to-world (ENU) matrix.
 *     (Orientation updates arrive slower than accel/gyro; newest matrix is reused —
 *     documented zero-order hold.)
 *  3. Gravity removal: linear = raw - R^T * (0,0,-g).
 *  4. Phone -> nav frame: a_world = R * linear.
 *  5. Integrate horizontal (east/north) accel -> velocity -> ENU displacement -> lat/lon.
 *  6. Heading = direction of velocity once speed > 0.5 m/s, else hold last.
 *  7. confidenceMeters grows ~linearly with time (no GNSS corrections in baseline).
 *
 * Vertical channel is damped (baro-less phone z-accel diverges in seconds); altitude
 * is held near its initial value. GNSS input is IGNORED (see interface docs).
 */
class BaselineDeadReckoningEngine : DeadReckoningEngine {

    private var state: NavigationState? = null
    private var lastImuTimeNanos: Long = -1L
    private var velEast = 0f
    private var velNorth = 0f
    private var headingDeg = 0f
    private var elapsedS = 0.0

    private val alignment = VehicleAlignmentModule()

    // Scratch buffers — no per-sample allocation on the hot path.
    private val rotM = FloatArray(9)
    private var hasOrientation = false
    private val accPhone = FloatArray(3)
    private val gravPhone = FloatArray(3)
    private val linPhone = FloatArray(3)
    private val accWorld = FloatArray(3)

    override val currentState: NavigationState? get() = state

    override fun initialize(initialState: NavigationState) {
        reset()
        state = initialState
        velEast = initialState.velEast
        velNorth = initialState.velNorth
        headingDeg = initialState.headingDeg
    }

    override fun processImu(sample: ImuSample): NavigationState {
        val prev = state ?: return NavigationState(
            timestampNanos = sample.timestampNanos,
            latitude = 0.0, longitude = 0.0
        )

        // Orientation update (zero-order hold between rotation-vector events).
        if (sample.quatW != null && sample.quatX != null &&
            sample.quatY != null && sample.quatZ != null
        ) {
            val m = CoordinateTransformer.rotationMatrixFromQuaternion(
                sample.quatX, sample.quatY, sample.quatZ, sample.quatW
            )
            m.copyInto(rotM)
            if (!hasOrientation) {
                hasOrientation = true
                val ypr = CoordinateTransformer.yawPitchRollDeg(rotM)
                alignment.beginCalibration(ypr[0])
            }
        }

        // Timestep in seconds from hardware timestamps.
        var dt = 0.01
        if (lastImuTimeNanos > 0) {
            dt = (sample.timestampNanos - lastImuTimeNanos) / 1e9
            if (dt <= 0.0 || dt > 0.2) dt = 0.01 // glitch/gap guard
        }
        lastImuTimeNanos = sample.timestampNanos
        elapsedS += dt

        var aEast = 0f
        var aNorth = 0f
        if (hasOrientation) {
            accPhone[0] = sample.accelX; accPhone[1] = sample.accelY; accPhone[2] = sample.accelZ
            CoordinateTransformer.gravityInPhoneFrame(rotM, gravPhone)
            linPhone[0] = accPhone[0] - gravPhone[0]
            linPhone[1] = accPhone[1] - gravPhone[1]
            linPhone[2] = accPhone[2] - gravPhone[2]
            CoordinateTransformer.transform(linPhone, rotM, accWorld)
            aEast = accWorld[0]
            aNorth = accWorld[1]
            // NOTE: no low-pass / ZUPT / bias estimation here — pure baseline.
        }

        velEast += (aEast * dt).toFloat()
        velNorth += (aNorth * dt).toFloat()
        val dEast = velEast * dt
        val dNorth = velNorth * dt

        val (newLat, newLon) = GeoUtils.moveEnu(prev.latitude, prev.longitude, dNorth, dEast)

        val speed = kotlin.math.hypot(velEast.toDouble(), velNorth.toDouble()).toFloat()
        if (speed > 0.5f) {
            val worldYaw = GeoUtils.normalizeHeading(
                Math.toDegrees(atan2(velEast.toDouble(), velNorth.toDouble())).toFloat()
            )
            headingDeg = alignment.vehicleHeadingDeg(worldYaw)
        }

        // Uncertainty grows without bound absent GNSS: ~2 m + 3%/s heuristic for display only.
        val confidence = (2.0 + 0.5 * elapsedS).toFloat()

        val next = prev.copy(
            timestampNanos = sample.timestampNanos,
            latitude = newLat,
            longitude = newLon,
            velEast = velEast,
            velNorth = velNorth,
            headingDeg = headingDeg,
            confidenceMeters = confidence
        )
        state = next
        return next
    }

    override fun reset() {
        state = null
        lastImuTimeNanos = -1L
        velEast = 0f; velNorth = 0f
        headingDeg = 0f
        elapsedS = 0.0
        hasOrientation = false
        alignment.reset()
    }
}
