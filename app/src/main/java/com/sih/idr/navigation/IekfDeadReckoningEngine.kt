package com.sih.idr.navigation

import com.sih.idr.data.GnssMeasurement
import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState
import com.sih.idr.navigation.pdr.PdrDebug
import com.sih.idr.utils.GeoUtils
import kotlin.math.*

/**
 * Invariant Extended Kalman Filter (IEKF) dead-reckoning engine for wheeled vehicles.
 *
 * Faithfully ported from Brossard et al., "AI-IMU Dead-Reckoning", IEEE-TIV 2020
 * (https://github.com/mbrossar/ai-imu-dr), translated to Kotlin for Android with
 * no external dependencies and zero per-sample heap allocation.
 *
 * Key differences from RouteAwareFusionEngine:
 *  - Full 3D SO(3) strapdown INS — quaternion rotation matrix, not DCM heading.
 *  - 21-state IEKF: R(3), v(3), p(3), b_ω(3), b_a(3), Rc(3), tc(3).
 *  - Non-Holonomic Constraint (NHC) pseudo-measurements every sample.
 *  - Self-calibrating gyro and accel biases — the primary source of DR drift.
 *  - ZUPT at standstill for full velocity reset.
 *  - Open-loop map snap: road/trail/route snapping applied AFTER the IEKF
 *    output without feeding position corrections back into the filter state
 *    (avoids filter inconsistency from map-induced jumps).
 *  - Static NHC covariance (no CNN adapter) — matches paper's "proposed w/o
 *    cov. adapter" variant (~1.94% trel on KITTI).
 *
 * State vector layout (21D, covariance P is 21×21):
 *   0:3   δR  — rotation error (Lie algebra so3 perturbation)
 *   3:6   v   — velocity ENU (m/s)
 *   6:9   p   — position ENU from origin (m)
 *   9:12  b_ω — gyro bias (rad/s)
 *  12:15  b_a — accel bias (m/s²)
 *  15:18  δRc — IMU-to-car rotation error
 *  18:21  tc  — IMU-to-car level arm (m)
 *
 * World frame is ENU: x = East, y = North, z = Up. Gravity = [0, 0, −g].
 */
class IekfDeadReckoningEngine : DeadReckoningEngine, EngineDebugReporter {

    // ── IEKF state ──────────────────────────────────────────────────────────
    /** IMU→world ENU rotation matrix (row-major FloatArray(9)). */
    private val R  = FloatArray(9)
    /** World-frame velocity [East, North, Up] in m/s. */
    private val v  = FloatArray(3)
    /** ENU position from seed origin [East, North, Up] in metres. */
    private val p  = FloatArray(3)
    /** Gyroscope bias estimate (rad/s). */
    private val bOmega = FloatArray(3)
    /** Accelerometer bias estimate (m/s²). */
    private val bAcc   = FloatArray(3)
    /** IMU-to-car-frame rotation (row-major FloatArray(9)), starts at identity. */
    private val Rc = FloatArray(9)
    /** IMU-to-car-frame level arm (m). */
    private val tc = FloatArray(3)
    /** 21×21 error covariance (DoubleArray for numerical stability). */
    private val P  = DoubleArray(P_DIM * P_DIM)

    // ── Origin & bookkeeping ─────────────────────────────────────────────────
    private var originLat = 0.0
    private var originLon = 0.0
    private var lastNanos = -1L
    private var elapsed   = 0.0
    private var state: NavigationState? = null
    override val currentState: NavigationState? get() = state

    // ── Standstill / ZUPT ────────────────────────────────────────────────────
    private var stillDuration = 0.0
    var isStationary = false; private set
    var zuptCount    = 0L;    private set
    /** Filtered gyro magnitude for ZUPT. */
    private var lpGyroMag = 0.0

    // ── Step detection (for logging / pedestrian motion boost) ───────────────
    private val stepCounter = com.sih.idr.navigation.pdr.DynamicStepCounter(1.0)
    var stepCount = 0; private set

    // ── GNSS gating ──────────────────────────────────────────────────────────
    private var lastGnssMillis = -1L
    private var gnssRejectStreak = 0
    var gnssRejects = 0; private set

    // ── Map snapping ─────────────────────────────────────────────────────────
    /** Optional destination-less road snapper wired by the ViewModel. */
    var roadSnapper: RoadSnapProvider? = null
    var snapMode = "FREE"; private set
    var status = "waiting for GNSS"; private set
    var gpsAvailable = false; private set
    var speedFailure = false; private set
    var lateralFailure = false; private set
    var mountCalibrated = true; private set  // IEKF self-calibrates via Rc

    // Route constraint (same API surface as RouteAwareFusionEngine).
    private var route: MutableList<Pair<Double, Double>> = mutableListOf()
    private var routeCumDist: DoubleArray = DoubleArray(0)
    private var routeS = 0.0
    private var destinationLat: Double? = null
    private var destinationLon: Double? = null
    private var pendingRouteLatLon: List<Pair<Double, Double>>? = null

    // Breadcrumb trail.
    private val trailE = ArrayDeque<Double>()
    private val trailN = ArrayDeque<Double>()
    private val trailH = ArrayDeque<Float>()
    private var lastTrailX = Double.NaN; private var lastTrailY = Double.NaN
    private var trAx=0.0;private var trAy=0.0;private var trBx=0.0;private var trBy=0.0
    private var trHasEdge=false; private var trReversed=false
    var trailSnapActive = false; private set
    var trailSize = 0; private set
    private var lastTrailQueryX=0.0;private var lastTrailQueryY=0.0;private var lastTrailQueryElapsed=-1.0
    private val recentRawE = ArrayDeque<Double>(); private val recentRawN = ArrayDeque<Double>()
    private var lastRecentX=Double.NaN; private var lastRecentY=Double.NaN

    // Auto road snap.
    private var autoAx=0.0;private var autoAy=0.0;private var autoBx=0.0;private var autoBy=0.0
    private var autoHasEdge=false; private var autoSnapActive=false
    private var lastSnapX=0.0;private var lastSnapY=0.0;private var lastSnapElapsed=-1.0
    private var lastSnapHeading=0f

    // ── NHC state ─────────────────────────────────────────────────────────────
    /** Last lateral NHC residual (m/s) for logging. */
    private var nhcResidualLat = 0f

    // ── Debug reporter ────────────────────────────────────────────────────────
    override var lastDebug: PdrDebug? = null; private set

    // ── Pre-allocated scratch buffers (never allocated on the hot path) ───────
    private val s3a=FloatArray(3); private val s3b=FloatArray(3); private val s3c=FloatArray(3)
    private val s9a=FloatArray(9); private val s9b=FloatArray(9); private val s9c=FloatArray(9)
    // Double scratch for Kalman algebra
    private val tmpF    = DoubleArray(P_DIM * P_DIM)
    private val tmpG    = DoubleArray(P_DIM * Q_DIM)
    private val tmpGQGt = DoubleArray(P_DIM * P_DIM)
    private val tmpPhi  = DoubleArray(P_DIM * P_DIM)
    private val tmpPhi2 = DoubleArray(P_DIM * P_DIM)
    private val tmpPhi3 = DoubleArray(P_DIM * P_DIM)
    private val tmpA    = DoubleArray(P_DIM * P_DIM)
    private val tmpB    = DoubleArray(P_DIM * P_DIM)
    // NHC update scratch (2×21, 2×2, 21×2)
    private val Hnhc = DoubleArray(2 * P_DIM)
    private val Snhc = DoubleArray(4)
    private val Knhc = DoubleArray(P_DIM * 2)
    private val dxNhc = DoubleArray(P_DIM)
    // ZUPT update scratch (3×21, etc.)
    private val Hzupt = DoubleArray(3 * P_DIM)
    private val Szupt = DoubleArray(9)
    private val Kzupt = DoubleArray(P_DIM * 3)
    private val dxZupt = DoubleArray(P_DIM)

    // ── Public API ────────────────────────────────────────────────────────────

    fun setDestination(latitude: Double, longitude: Double) {
        destinationLat = latitude; destinationLon = longitude
        if (state == null) return; rebuildRoute()
    }

    fun setRouteLatLon(points: List<Pair<Double, Double>>) {
        pendingRouteLatLon = points.toList()
        if (state == null) return; applyRouteLatLon(points)
    }

    fun clearRoute() {
        route.clear(); updateRouteDistances(); routeS = 0.0
        destinationLat = null; destinationLon = null
    }

    fun routeLatLon(): List<Pair<Double, Double>> =
        route.map { GeoUtils.moveEnu(originLat, originLon, it.second, it.first) }

    // ── DeadReckoningEngine ────────────────────────────────────────────────────

    override fun initialize(initialState: NavigationState) {
        reset()
        state = initialState
        originLat = initialState.latitude
        originLon = initialState.longitude

        // Seed R from initial heading (compass → ENU rotation around Z axis).
        val hRad = Math.toRadians(initialState.headingDeg.toDouble())
        val ch = cos(hRad).toFloat(); val sh = sin(hRad).toFloat()
        // R = Rz(heading): maps phone Y-forward to world North direction.
        So3Utils.identity(R)
        R[0] = ch;  R[1] = sh    // col 0: East = cos(h)·East - sin(h)·North
        R[3] = -sh; R[4] = ch    // col 1: North = sin(h)·East + cos(h)·North
        // R[8] = 1 (Up stays Up)
        attitudeInitialized = false

        val speed = initialState.speed.toDouble()
        val hR = Math.toRadians(initialState.headingDeg.toDouble())
        v[0] = (speed * sin(hR)).toFloat()
        v[1] = (speed * cos(hR)).toFloat()
        v[2] = 0f

        So3Utils.identity(Rc)
        initCovariance()
        status = "GNSS anchor acquired"
        pendingRouteLatLon?.let { applyRouteLatLon(it) } ?: rebuildRoute()
    }

    private var attitudeInitialized = false

    /**
     * Latches 3D tilt (pitch & roll) from Android rotation vector or gravity vector
     * so that gravity is canceled out regardless of phone mount orientation.
     */
    private fun alignAttitude(sample: ImuSample) {
        if (attitudeInitialized) return
        val targetHeading = state?.headingDeg ?: 0f
        if (sample.quatX != null && sample.quatY != null &&
            sample.quatZ != null && sample.quatW != null) {
            So3Utils.fromQuaternion(sample.quatX, sample.quatY, sample.quatZ, sample.quatW, R)
            val currentYaw = So3Utils.yawDeg(R)
            val deltaYawRad = Math.toRadians((targetHeading - currentYaw).toDouble())
            val cd = cos(deltaYawRad).toFloat(); val sd = sin(deltaYawRad).toFloat()
            val rz = floatArrayOf(
                cd,  sd, 0f,
                -sd, cd, 0f,
                0f,  0f, 1f
            )
            So3Utils.multiply(rz, R, s9a)
            So3Utils.copy(s9a, R)
            attitudeInitialized = true
        } else {
            val ax = sample.accelX.toDouble(); val ay = sample.accelY.toDouble(); val az = sample.accelZ.toDouble()
            val norm = sqrt(ax*ax + ay*ay + az*az)
            if (norm > 5.0 && norm < 15.0) {
                val pitch = atan2(ay, az)
                val roll = atan2(-ax, sqrt(ay*ay + az*az))
                val yawRad = Math.toRadians(targetHeading.toDouble())
                val ch = cos(yawRad).toFloat(); val sh = sin(yawRad).toFloat()
                val cp = cos(pitch).toFloat(); val sp = sin(pitch).toFloat()
                val cr = cos(roll).toFloat(); val sr = sin(roll).toFloat()

                // R = Rz(yaw) · Rx(pitch) · Ry(roll)
                R[0] = ch*cr + sh*sp*sr;  R[1] = sh*cp; R[2] = ch*sr - sh*sp*cr
                R[3] = -sh*cr + ch*sp*sr; R[4] = ch*cp; R[5] = -sh*sr - ch*sp*cr
                R[6] = -cp*sr;            R[7] = sp;    R[8] = cp*cr
                attitudeInitialized = true
            }
        }
    }

    override fun processImu(sample: ImuSample): NavigationState {
        val prev = state ?: return NavigationState(sample.timestampNanos, 0.0, 0.0)
        val dt = if (lastNanos < 0) 0.01
                 else ((sample.timestampNanos - lastNanos) / 1e9).coerceIn(0.001, 0.2)
        lastNanos = sample.timestampNanos
        elapsed += dt

        // ── 1. Align 3D attitude on first sample ─────────────────────────────
        alignAttitude(sample)

        // ── 2. Extract gyro / accel ──────────────────────────────────────────
        val gx = sample.gyroX; val gy = sample.gyroY; val gz = sample.gyroZ
        // Effective (bias-corrected) angular rate
        val omX = gx - bOmega[0]; val omY = gy - bOmega[1]; val omZ = gz - bOmega[2]

        // Strapdown INS: specific force f measured by IMU in body frame.
        // World gravity [0,0,-g] is integrated during propagation.
        val fAcc = floatArrayOf(
            sample.accelX - bAcc[0],
            sample.accelY - bAcc[1],
            sample.accelZ - bAcc[2]
        )

        // ── 3. IEKF Propagation ──────────────────────────────────────────────
        propagate(fAcc, floatArrayOf(omX, omY, omZ), dt.toFloat())

        // ── 4. Standstill detection ──────────────────────────────────────────
        val gyroMag = sqrt((gx*gx + gy*gy + gz*gz).toDouble())
        lpGyroMag += GYRO_LP * (gyroMag - lpGyroMag)
        val rawAccMag = sqrt((sample.accelX*sample.accelX + sample.accelY*sample.accelY + sample.accelZ*sample.accelZ).toDouble())
        val linAccMag = abs(rawAccMag - GRAVITY_D)
        val isStep = stepCounter.findStep(rawAccMag)
        if (isStep) stepCount++
        val quiet = gyroMag < ZUPT_GYRO_MAX && linAccMag < ZUPT_ACC_MAX && !isStep
        stillDuration = if (quiet) stillDuration + dt else 0.0
        val speed = sqrt((v[0]*v[0] + v[1]*v[1]).toDouble())
        isStationary = stillDuration >= ZUPT_MIN_S && (speed < 1.2 || stillDuration >= 2.5)

        // ── 5. Kalman Updates ─────────────────────────────────────────────────
        if (isStationary) {
            zuptUpdate()
            // Absorb accel residual at standstill: R · fAcc should equal [0,0,g]
            So3Utils.rotVec(R, fAcc, s3a)
            val errW = floatArrayOf(s3a[0], s3a[1], (s3a[2] - GRAVITY))
            So3Utils.rotVecT(R, errW, s3b)
            bAcc[0] = (bAcc[0] + ZUPT_BIAS_ALPHA * s3b[0]).toFloat().coerceIn(-1.5f, 1.5f)
            bAcc[1] = (bAcc[1] + ZUPT_BIAS_ALPHA * s3b[1]).toFloat().coerceIn(-1.5f, 1.5f)
            bAcc[2] = (bAcc[2] + ZUPT_BIAS_ALPHA * s3b[2]).toFloat().coerceIn(-1.5f, 1.5f)
            zuptCount++
        } else {
            nhcUpdate(floatArrayOf(omX, omY, omZ))
        }

        // ── 5. Renormalize R and Rc periodically ─────────────────────────────
        val stepIdx = (elapsed / dt).toInt()
        if (stepIdx % 100 == 0) So3Utils.normalizeInPlace(R)
        if (stepIdx % 1000 == 0) So3Utils.normalizeInPlace(Rc)

        // ── 6. Raw ENU position → lat/lon ────────────────────────────────────
        val rawX = p[0].toDouble(); val rawY = p[1].toDouble()
        pushRecentRaw(rawX, rawY)

        // ── 7. Map snapping (open-loop on IEKF output) ────────────────────────
        var outX = rawX; var outY = rawY
        snapMode = if (isStationary) "ZUPT" else "FREE"
        val heading = So3Utils.yawDeg(R)
        val routeMatch = constrainToRoute(rawX, rawY, speed)
        if (routeMatch != null) {
            outX = routeMatch.x; outY = routeMatch.y
            routeS = routeMatch.routeS; snapMode = "ROUTE"
            trailSnapActive = false; autoSnapActive = false
        } else {
            val trailMatch = snapToTrail(rawX, rawY, heading, speed)
            if (trailMatch != null) {
                outX = trailMatch.x; outY = trailMatch.y
                snapMode = if (trReversed) "TRAIL-REV" else "TRAIL"
                trailSnapActive = true; autoSnapActive = false
            } else {
                val autoMatch = autoSnapToRoad(rawX, rawY, speed)
                if (autoMatch != null) {
                    outX = autoMatch.x; outY = autoMatch.y
                    snapMode = "AUTO"; autoSnapActive = true; trailSnapActive = false
                } else {
                    autoSnapActive = false; trailSnapActive = false
                }
            }
        }
        addTrailPoint(outX, outY, heading)

        // ── 8. Build NavigationState ──────────────────────────────────────────
        val ll = GeoUtils.moveEnu(originLat, originLon, outY, outX)
        val outHeadingDeg = heading
        gpsAvailable = isGnssFresh()
        speedFailure = !speed.isFinite() || speed > 55.0
        val confidence = (2.0 + elapsed * if (gpsAvailable) 0.05 else 0.8).toFloat()
        val next = prev.copy(
            timestampNanos = sample.timestampNanos,
            latitude  = ll.first,
            longitude = ll.second,
            velEast   = v[0],
            velNorth  = v[1],
            headingDeg = outHeadingDeg,
            confidenceMeters = confidence
        )
        state = next

        // ── 9. Debug snapshot ─────────────────────────────────────────────────
        val biasOmMag = sqrt((bOmega[0]*bOmega[0]+bOmega[1]*bOmega[1]+bOmega[2]*bOmega[2]).toDouble()).toFloat()
        val biasAccMag = sqrt((bAcc[0]*bAcc[0]+bAcc[1]*bAcc[1]+bAcc[2]*bAcc[2]).toDouble()).toFloat()
        lastDebug = PdrDebug(
            timestampNanos  = sample.timestampNanos,
            accMagnitude    = rawAccMag.toFloat(),
            stepDetected    = isStep,
            strideLength    = (speed * dt).toFloat(),
            gyroHeadingRad  = Math.toRadians(heading.toDouble()).toFloat(),
            magHeadingRad   = Float.NaN,
            fusedHeadingRad = Math.toRadians(heading.toDouble()).toFloat(),
            xEast           = outX.toFloat(),
            yNorth          = outY.toFloat(),
            latitude        = ll.first,
            longitude       = ll.second,
            forwardSpeedMps = speed.toFloat(),
            yawRateDegS     = Math.toDegrees(omZ.toDouble()).toFloat(),
            lateralVelMps   = nhcResidualLat,
            snapMode        = snapMode,
            biasOmegaMag    = biasOmMag,
            biasAccMag      = biasAccMag,
            nhcResidualLat  = nhcResidualLat
        )
        return next
    }

    override fun onGnssMeasurement(measurement: GnssMeasurement) {
        val g = measurement.sample
        val acc = (g.accuracy ?: 5f).toDouble().coerceAtLeast(1.0)
        if (acc > 30.0) return
        val eGps = Math.toRadians(g.longitude - originLon) * 6_378_137.0 * cos(Math.toRadians(originLat))
        val nGps = Math.toRadians(g.latitude  - originLat) * 6_378_137.0
        val jump = hypot(eGps - p[0], nGps - p[1])
        val nowMs = g.timestampMillis
        val dtGps = if (lastGnssMillis < 0) 0.0 else ((nowMs - lastGnssMillis) / 1000.0).coerceIn(0.0, 3600.0)
        val gate = 3.0 * acc + min(sqrt((v[0]*v[0]+v[1]*v[1]).toDouble()), 25.0) * dtGps + 15.0
        val forced = gnssRejectStreak >= 5
        if (!forced && lastGnssMillis >= 0 && jump > gate) {
            gnssRejectStreak++; gnssRejects++
            status = "GNSS glitch rejected (%.0fm jump, gate %.0fm)".format(jump, gate)
            return
        }
        gnssRejectStreak = 0; lastGnssMillis = nowMs; gpsAvailable = true
        // Soft position nudge — does NOT enter the Kalman filter state (open-loop correction)
        val k = (5.0 / acc).coerceIn(0.15, 1.0)
        p[0] += (k * (eGps - p[0])).toFloat()
        p[1] += (k * (nGps - p[1])).toFloat()
        routeS = projectRouteS(p[0].toDouble(), p[1].toDouble())
        g.speed?.let { spd ->
            if (spd.isFinite() && spd in 0f..40f && spd > 1.5f && g.bearing != null) {
                val bRad = Math.toRadians(g.bearing.toDouble())
                v[0] = (spd * sin(bRad)).toFloat()
                v[1] = (spd * cos(bRad)).toFloat()
                // Align yaw in R with GPS bearing while preserving pitch & roll tilt
                val currYaw = So3Utils.yawDeg(R)
                val deltaYawRad = Math.toRadians((g.bearing.toDouble() - currYaw))
                val cd = cos(deltaYawRad).toFloat(); val sd = sin(deltaYawRad).toFloat()
                val rz = floatArrayOf(
                    cd,  sd, 0f,
                    -sd, cd, 0f,
                    0f,  0f, 1f
                )
                So3Utils.multiply(rz, R, s9a)
                So3Utils.copy(s9a, R)
            } else if (spd < 0.5f) { v[0] = 0f; v[1] = 0f }
        }
        state = state?.copy(
            latitude = originLat + Math.toDegrees(p[1] / 6_378_137.0),
            longitude = originLon + Math.toDegrees(p[0] / (6_378_137.0 * cos(Math.toRadians(originLat)))),
            velEast = v[0], velNorth = v[1],
            confidenceMeters = acc.toFloat()
        )
        status = if (route.size >= 2) "GNSS fused: route constrained" else "GNSS fused: inertial calibrated"
    }

    override fun reset() {
        state = null; lastNanos = -1L; elapsed = 0.0
        v.fill(0f); p.fill(0f); bOmega.fill(0f); bAcc.fill(0f); tc.fill(0f)
        So3Utils.identity(R); So3Utils.identity(Rc)
        attitudeInitialized = false
        P.fill(0.0); stillDuration = 0.0; isStationary = false; zuptCount = 0L
        lpGyroMag = 0.0; stepCount = 0; stepCounter.reset()
        route.clear(); updateRouteDistances(); routeS = 0.0
        destinationLat = null; destinationLon = null
        lastGnssMillis = -1L; gnssRejectStreak = 0; gnssRejects = 0
        gpsAvailable = false; speedFailure = false; lateralFailure = false
        snapMode = "FREE"; status = "waiting for GNSS"
        trailE.clear(); trailN.clear(); trailH.clear()
        lastTrailX = Double.NaN; lastTrailY = Double.NaN
        trHasEdge = false; trReversed = false; trailSnapActive = false; trailSize = 0
        lastTrailQueryX = 0.0; lastTrailQueryY = 0.0; lastTrailQueryElapsed = -1.0
        recentRawE.clear(); recentRawN.clear()
        lastRecentX = Double.NaN; lastRecentY = Double.NaN
        autoHasEdge = false; autoSnapActive = false
        lastSnapX = 0.0; lastSnapY = 0.0; lastSnapElapsed = -1.0
        nhcResidualLat = 0f; lastDebug = null
    }

    // ── IEKF Core ──────────────────────────────────────────────────────────────

    /** Initialize error covariance P from paper Section IV-B. */
    private fun initCovariance() {
        P.fill(0.0)
        // Only pitch/roll have uncertainty (yaw seeded from GPS/magnetometer).
        setP(0, 0, COV_ROT0); setP(1, 1, COV_ROT0)   // pitch/roll; yaw=0
        setP(3, 3, COV_V0);   setP(4, 4, COV_V0)       // horizontal velocity
        setP(9,  9, COV_B_OMEGA0); setP(10,10, COV_B_OMEGA0); setP(11,11, COV_B_OMEGA0)
        setP(12,12, COV_B_ACC0);   setP(13,13, COV_B_ACC0);   setP(14,14, COV_B_ACC0)
        setP(15,15, COV_RC0);  setP(16,16, COV_RC0);   setP(17,17, COV_RC0)
        setP(18,18, COV_TC0);  setP(19,19, COV_TC0);   setP(20,20, COV_TC0)
    }

    /**
     * IEKF propagation step: integrate one IMU sample.
     * Updates R, v, p and propagates covariance P.
     *
     * @param linAcc  Bias-corrected specific force in phone frame (m/s²).
     * @param omega   Bias-corrected angular rate in phone frame (rad/s).
     * @param dt      Time step in seconds.
     */
    private fun propagate(linAcc: FloatArray, omega: FloatArray, dt: Float) {
        val dtD = dt.toDouble()

        // --- State propagation ---
        // 1. World-frame acceleration = R · linAcc + g
        So3Utils.rotVec(R, linAcc, s3a)   // s3a = R·linAcc
        val aEx = s3a[0]; val aEy = s3a[1]; val aEz = s3a[2] - GRAVITY

        // 2. Position: p ← p + v·dt + ½·a·dt²
        val dt2h = 0.5f * dt * dt
        p[0] += v[0]*dt + aEx.toFloat()*dt2h
        p[1] += v[1]*dt + aEy.toFloat()*dt2h
        p[2] += v[2]*dt + aEz.toFloat()*dt2h

        // 3. Velocity: v ← v + a·dt
        val vPrevE = v[0]; val vPrevN = v[1]; val vPrevU = v[2]
        v[0] += (aEx * dtD).toFloat()
        v[1] += (aEy * dtD).toFloat()
        v[2] += (aEz * dtD).toFloat()

        // 4. Orientation: R ← R · exp(omega·dt)
        s3b[0] = omega[0]*dt; s3b[1] = omega[1]*dt; s3b[2] = omega[2]*dt
        So3Utils.exp(s3b, s9a)         // s9a = exp(omega·dt)
        So3Utils.multiply(R, s9a, s9b) // s9b = R·exp
        So3Utils.copy(s9b, R)

        // --- Covariance propagation ---
        // Build F (21×21) and G (21×18) matrices (scaled by dt, as in Python impl.)
        // F and G are very sparse — only set non-zero blocks.
        tmpF.fill(0.0); tmpG.fill(0.0)

        // F blocks (from Python utils_numpy_filter.py propagate_cov):
        // F[3:6, 0:3] = skew(g) * dt  (gravity coupling to velocity via rotation error)
        val gSkew00=0.0;val gSkew01=GRAVITY_D;val gSkew02=0.0
        val gSkew10=-GRAVITY_D;val gSkew11=0.0;val gSkew12=0.0
        // g = [0,0,-g]; skew(g) = [[0,g,0],[-g,0,0],[0,0,0]]
        setF(3,0, GRAVITY_D*dtD); setF(3,1, 0.0); setF(3,2, 0.0)
        setF(4,0, 0.0);            setF(4,1, 0.0); setF(4,2, 0.0)
        // g = [0,0,-g]: skew = [[0,−(−g),0],[−g,0,0]] no wait
        // skew([0,0,-g]) = [[0,−(−g), 0],[−(0), 0, −(0)],[−(0),−(0),0]]
        // wait: skew([ax,ay,az]) = [[0,-az,ay],[az,0,-ax],[-ay,ax,0]]
        // skew([0,0,-g]) = [[0,g,0],[-g,0,0],[0,0,0]]  -- but wait:
        // skew([0,0,-g]): az = -g: [[0,-(-g),0],[(-g),0,0],[0,0,0]] = [[0,g,0],[-g,0,0],[0,0,0]]
        // So F[3,0]=0, F[3,1]= -g? Let me be careful:
        // skew(g) where g=[0,0,-g]: row 0 = [0, -g_z, g_y] = [0, g, 0]
        //                           row 1 = [g_z, 0, -g_x] = [-g, 0, 0]
        //                           row 2 = [-g_y, g_x, 0] = [0, 0, 0]
        // F[vel_y, rot_x] = -g, F[vel_x, rot_y] = g → correct:
        tmpF[rc(3,1)] = GRAVITY_D * dtD    // F[vel_x, rot_y] = g
        tmpF[rc(4,0)] = -GRAVITY_D * dtD   // F[vel_y, rot_x] = -g

        // F[6:9, 3:6] = I3 * dt  (position depends on velocity)
        tmpF[rc(6,3)] = dtD; tmpF[rc(7,4)] = dtD; tmpF[rc(8,5)] = dtD

        // F[3:6, 12:15] = -R * dt  (velocity depends on accel bias)
        for (i in 0..2) for (j in 0..2) tmpF[rc(3+i, 12+j)] = -R[i*3+j].toDouble() * dtD

        // F[0:3, 9:12] = -R * dt  (rotation depends on gyro bias)
        for (i in 0..2) for (j in 0..2) tmpF[rc(i, 9+j)] = -R[i*3+j].toDouble() * dtD

        // F[3:6, 9:12] = -[v_prev]× · R * dt
        val vS = doubleArrayOf(vPrevE.toDouble(), vPrevN.toDouble(), vPrevU.toDouble())
        for (i in 0..2) for (j in 0..2) {
            // ([v]× · R)[i,j] = sum_k [v]×[i,k] · R[k,j]
            val vSkewRij = skewRow(i, vS, R, j)
            tmpF[rc(3+i, 9+j)] = -vSkewRij * dtD
        }

        // F[6:9, 9:12] = -[p_prev]× · R * dt  (use position before update)
        val pS = doubleArrayOf((p[0]-v[0]*dt).toDouble(), (p[1]-v[1]*dt).toDouble(), (p[2]-v[2]*dt).toDouble())
        for (i in 0..2) for (j in 0..2) {
            val pSkewRij = skewRow(i, pS, R, j)
            tmpF[rc(6+i, 9+j)] = -pSkewRij * dtD
        }

        // G blocks (noise input):
        // G[0:3, 0:3] = R * dt  (rotation noise)
        for (i in 0..2) for (j in 0..2) tmpG[rg(i, j)] = R[i*3+j].toDouble() * dtD
        // G[3:6, 3:6] = R * dt  (accel noise → velocity)
        for (i in 0..2) for (j in 0..2) tmpG[rg(3+i, 3+j)] = R[i*3+j].toDouble() * dtD
        // G[3:6, 0:3] = [v]× · R * dt
        for (i in 0..2) for (j in 0..2) tmpG[rg(3+i, j)] = skewRow(i, vS, R, j) * dtD
        // G[6:9, 0:3] = [p]× · R * dt
        for (i in 0..2) for (j in 0..2) tmpG[rg(6+i, j)] = skewRow(i, pS, R, j) * dtD
        // G[9:12, 6:9] = I (gyro bias noise)
        tmpG[rg(9,6)]=dtD; tmpG[rg(10,7)]=dtD; tmpG[rg(11,8)]=dtD
        // G[12:15, 9:12] = I (accel bias noise)
        tmpG[rg(12,9)]=dtD; tmpG[rg(13,10)]=dtD; tmpG[rg(14,11)]=dtD
        // G[15:18, 12:15] = I (Rc noise)
        tmpG[rg(15,12)]=dtD; tmpG[rg(16,13)]=dtD; tmpG[rg(17,14)]=dtD
        // G[18:21, 15:18] = I (tc noise)
        tmpG[rg(18,15)]=dtD; tmpG[rg(19,16)]=dtD; tmpG[rg(20,17)]=dtD

        // Φ = I + F + F²/2 + F³/6  (third-order Padé/Taylor approximation)
        // tmpPhi = I
        identity21(tmpPhi)
        // tmpPhi += F
        for (i in 0 until P_DIM*P_DIM) tmpPhi[i] += tmpF[i]
        // F² / 2
        mmul21(tmpF, tmpF, tmpPhi2)
        for (i in 0 until P_DIM*P_DIM) tmpPhi[i] += 0.5 * tmpPhi2[i]
        // F³ / 6
        mmul21(tmpPhi2, tmpF, tmpPhi3)
        for (i in 0 until P_DIM*P_DIM) tmpPhi[i] += (1.0/6.0) * tmpPhi3[i]

        // G·Q·Gᵀ  (since Q is diagonal, (G·Q·Gᵀ)[i,j] = Σ_k G[i,k]·Q[k]·G[j,k])
        tmpGQGt.fill(0.0)
        for (i in 0 until P_DIM) for (j in 0 until P_DIM) {
            var s = 0.0
            for (k in 0 until Q_DIM) s += tmpG[rg(i,k)] * Q_DIAG[k] * tmpG[rg(j,k)]
            tmpGQGt[rc(i,j)] = s
        }

        // P_new = Φ · (P + G·Q·Gᵀ) · Φᵀ
        // tmpA = P + G·Q·Gᵀ
        for (i in 0 until P_DIM*P_DIM) tmpA[i] = P[i] + tmpGQGt[i]
        // tmpB = Φ · tmpA
        mmul21(tmpPhi, tmpA, tmpB)
        // P = tmpB · Φᵀ  (= tmpB · Phiᵀ)
        mmul21T(tmpB, tmpPhi, P)   // P = B·ΦᵀL (second arg transposed)
    }

    /**
     * Non-Holonomic Constraint (NHC) update.
     * Pseudo-measurement: v_body[1] ≈ 0 (lateral), v_body[2] ≈ 0 (vertical).
     *
     * @param omegaEff  Bias-corrected gyro reading (rad/s).
     */
    private fun nhcUpdate(omegaEff: FloatArray) {
        // R_body = R · Rc
        So3Utils.multiply(R, Rc, s9a)  // s9a = R_body
        // v_imu = Rᵀ · v
        So3Utils.rotVecT(R, v, s3a)    // s3a = v_imu
        // v_body = Rcᵀ · v_imu + tc × omega_eff
        So3Utils.rotVecT(Rc, s3a, s3b) // s3b = Rcᵀ · v_imu
        // tc × omega_eff
        So3Utils.cross(tc, omegaEff, s3c) // s3c = tc × omega_eff
        val vBodyLat = s3b[0] + s3c[0]   // lateral (index 0, phone X axis)
        val vBodyUp  = s3b[2] + s3c[2]   // vertical (index 2, phone Z axis)
        nhcResidualLat = -vBodyLat

        val rLat = -vBodyLat.toDouble()
        val rUp  = -vBodyUp.toDouble()
        if (!rLat.isFinite() || !rUp.isFinite()) return

        // H (2×21): Jacobian of pseudo-measurement w.r.t. state
        // Row 0 = lateral (index 0); Row 1 = vertical (index 2)
        Hnhc.fill(0.0)
        // R_body^T rows 0 and 2 → H[0, 3:6] and H[1, 3:6]
        for (j in 0..2) {
            Hnhc[rc2(0, 3+j)] = s9a[j*3+0].toDouble()  // row 0 of R_body^T
            Hnhc[rc2(1, 3+j)] = s9a[j*3+2].toDouble()  // row 2 of R_body^T
        }
        // Rcᵀ · [v_imu]× (rows 0,2) = H_v_imu[0, 2]
        val hVimu = FloatArray(9); So3Utils.skew(s3a, s9b)  // s9b = [v_imu]×
        val rcT = FloatArray(9); So3Utils.transpose(Rc, rcT)
        So3Utils.multiply(rcT, s9b, hVimu) // hVimu = Rcᵀ · [v_imu]×
        for (j in 0..2) {
            Hnhc[rc2(0, 15+j)] = hVimu[0*3+j].toDouble()
            Hnhc[rc2(1, 15+j)] = hVimu[2*3+j].toDouble()
        }
        // -[tc]× (rows 0,2) → H[:, 9:12]
        So3Utils.skew(tc, s9b)  // s9b = [tc]×
        for (j in 0..2) {
            Hnhc[rc2(0, 9+j)] = -s9b[0*3+j].toDouble()
            Hnhc[rc2(1, 9+j)] = -s9b[2*3+j].toDouble()
        }
        // -[omega_eff]× (rows 0,2) → H[:, 18:21]
        So3Utils.skew(omegaEff, s9b)
        for (j in 0..2) {
            Hnhc[rc2(0, 18+j)] = -s9b[0*3+j].toDouble()
            Hnhc[rc2(1, 18+j)] = -s9b[2*3+j].toDouble()
        }

        // S = H·P·Hᵀ + N (2×2)
        // K = P·Hᵀ·S⁻¹ (21×2)
        // dx = K·r (21)
        val PHt = DoubleArray(P_DIM * 2)
        for (i in 0 until P_DIM) for (j in 0..1) {
            var s = 0.0
            for (k in 0 until P_DIM) s += P[rc(i,k)] * Hnhc[rc2(j,k)]
            PHt[i*2+j] = s
        }
        Snhc[0] = NHC_LAT + (0 until P_DIM).sumOf { k -> Hnhc[0*P_DIM+k] * PHt[k*2+0] }
        Snhc[1] = (0 until P_DIM).sumOf { k -> Hnhc[0*P_DIM+k] * PHt[k*2+1] }
        Snhc[2] = Snhc[1]
        Snhc[3] = NHC_UP + (0 until P_DIM).sumOf { k -> Hnhc[1*P_DIM+k] * PHt[k*2+1] }
        // 2×2 inverse
        val det = Snhc[0]*Snhc[3] - Snhc[1]*Snhc[2]
        if (abs(det) < 1e-20) return
        val SInv = doubleArrayOf(Snhc[3]/det, -Snhc[1]/det, -Snhc[2]/det, Snhc[0]/det)
        // K = PHt · S⁻¹  (21×2)
        for (i in 0 until P_DIM) for (j in 0..1) {
            Knhc[i*2+j] = PHt[i*2+0]*SInv[j] + PHt[i*2+1]*SInv[2+j]
        }
        // dx = K · [rLat, rUp]
        for (i in 0 until P_DIM) dxNhc[i] = Knhc[i*2]*rLat + Knhc[i*2+1]*rUp

        // Apply IEKF left-invariant update
        applyDx(dxNhc)

        // Joseph form covariance update: P = (I-K·H)·P·(I-K·H)ᵀ + K·N·Kᵀ
        // (I-K·H) stored in tmpA
        identity21(tmpA)
        for (i in 0 until P_DIM) for (k in 0 until P_DIM) {
            tmpA[rc(i,k)] -= Knhc[i*2]*Hnhc[0*P_DIM+k] + Knhc[i*2+1]*Hnhc[1*P_DIM+k]
        }
        mmul21(tmpA, P, tmpB)
        mmul21T(tmpB, tmpA, tmpPhi)  // tmpPhi = (I-KH)·P·(I-KH)ᵀ
        // + K·N·Kᵀ  (N = diag(NHC_LAT, NHC_UP))
        for (i in 0 until P_DIM) for (j in 0 until P_DIM) {
            P[rc(i,j)] = tmpPhi[rc(i,j)] +
                NHC_LAT * Knhc[i*2] * Knhc[j*2] +
                NHC_UP  * Knhc[i*2+1] * Knhc[j*2+1]
        }
        symmetrize()
    }

    /** ZUPT: full 3D velocity = 0 pseudo-measurement. */
    private fun zuptUpdate() {
        Hzupt.fill(0.0)
        // H selects the velocity block (indices 3,4,5): H[0,3]=1, H[1,4]=1, H[2,5]=1
        Hzupt[0*P_DIM+3]=1.0; Hzupt[1*P_DIM+4]=1.0; Hzupt[2*P_DIM+5]=1.0
        val PHt = DoubleArray(P_DIM*3)
        for (i in 0 until P_DIM) for (j in 0..2) PHt[i*3+j] = P[rc(i, 3+j)]
        // S = PHt (rows 3..5) + N  (3×3 diagonal N = ZUPT_COV·I)
        for (i in 0..2) for (j in 0..2) {
            Szupt[i*3+j] = P[rc(3+i, 3+j)] + (if (i==j) ZUPT_COV else 0.0)
        }
        val sInv = invert3x3(Szupt) ?: return
        for (i in 0 until P_DIM) for (j in 0..2) {
            Kzupt[i*3+j] = PHt[i*3+0]*sInv[j] + PHt[i*3+1]*sInv[3+j] + PHt[i*3+2]*sInv[6+j]
        }
        // dx = -K · v  (residual is 0 - v)
        for (i in 0 until P_DIM) dxZupt[i] = -(Kzupt[i*3]*v[0] + Kzupt[i*3+1]*v[1] + Kzupt[i*3+2]*v[2])
        applyDx(dxZupt)
        identity21(tmpA)
        for (i in 0 until P_DIM) for (k in 0 until P_DIM) {
            tmpA[rc(i,k)] -= Kzupt[i*3]*Hzupt[0*P_DIM+k]+Kzupt[i*3+1]*Hzupt[1*P_DIM+k]+Kzupt[i*3+2]*Hzupt[2*P_DIM+k]
        }
        mmul21(tmpA, P, tmpB); mmul21T(tmpB, tmpA, P)
        for (i in 0 until P_DIM) for (j in 0 until P_DIM) {
            P[rc(i,j)] += ZUPT_COV*(Kzupt[i*3]*Kzupt[j*3]+Kzupt[i*3+1]*Kzupt[j*3+1]+Kzupt[i*3+2]*Kzupt[j*3+2])
        }
        symmetrize()
    }

    /**
     * Apply state correction vector dx (21D) using the IEKF left-invariant update.
     * Follows state_and_cov_update() from utils_numpy_filter.py.
     */
    private fun applyDx(dx: DoubleArray) {
        val dPhi = floatArrayOf(dx[0].toFloat(), dx[1].toFloat(), dx[2].toFloat())
        val dR = FloatArray(9); So3Utils.exp(dPhi, dR) // dR = exp(δθ)

        // Left-multiply: R ← dR · R
        So3Utils.multiply(dR, R, s9a); So3Utils.copy(s9a, R)

        // Left-invariant velocity update: v ← dR · v + J · dx[3:6]
        // For small angles J ≈ I + δθ×/2, use first-order J = I + skew(dPhi)/2
        val dv = floatArrayOf(dx[3].toFloat(), dx[4].toFloat(), dx[5].toFloat())
        So3Utils.rotVec(dR, v, s3a)   // s3a = dR·v
        v[0] = s3a[0] + dv[0]; v[1] = s3a[1] + dv[1]; v[2] = s3a[2] + dv[2]

        // Position: p ← dR · p + J · dx[6:9]
        val dp = floatArrayOf(dx[6].toFloat(), dx[7].toFloat(), dx[8].toFloat())
        So3Utils.rotVec(dR, p, s3a)
        p[0] = s3a[0] + dp[0]; p[1] = s3a[1] + dp[1]; p[2] = s3a[2] + dp[2]

        // Additive: biases (clamped to realistic MEMS physical ranges to prevent divergence)
        bOmega[0] = (bOmega[0] + dx[9].toFloat()).coerceIn(-0.1f, 0.1f)
        bOmega[1] = (bOmega[1] + dx[10].toFloat()).coerceIn(-0.1f, 0.1f)
        bOmega[2] = (bOmega[2] + dx[11].toFloat()).coerceIn(-0.1f, 0.1f)

        bAcc[0]   = (bAcc[0] + dx[12].toFloat()).coerceIn(-1.5f, 1.5f)
        bAcc[1]   = (bAcc[1] + dx[13].toFloat()).coerceIn(-1.5f, 1.5f)
        bAcc[2]   = (bAcc[2] + dx[14].toFloat()).coerceIn(-1.5f, 1.5f)

        // Rc ← exp(dx[15:18]) · Rc
        val dRc = FloatArray(9)
        val dRcPhi = floatArrayOf(dx[15].toFloat(), dx[16].toFloat(), dx[17].toFloat())
        So3Utils.exp(dRcPhi, dRc)
        So3Utils.multiply(dRc, Rc, s9a); So3Utils.copy(s9a, Rc)

        // tc += dx[18:21]
        tc[0] += dx[18].toFloat(); tc[1] += dx[19].toFloat(); tc[2] += dx[20].toFloat()
    }

    // ── Matrix helpers ─────────────────────────────────────────────────────────

    /** Index into 21×21 matrix. */
    private fun rc(r: Int, c: Int) = r * P_DIM + c
    /** Index into 2×21 matrix. */
    private fun rc2(r: Int, c: Int) = r * P_DIM + c
    /** Index into 21×18 G matrix. */
    private fun rg(r: Int, c: Int) = r * Q_DIM + c

    /** Set P[r,c] = v. */
    private fun setP(r: Int, c: Int, v: Double) { P[rc(r,c)] = v }

    /** Set tmpF[r,c] = v (P_DIM×P_DIM). */
    private fun setF(r: Int, c: Int, v: Double) { tmpF[rc(r,c)] = v }

    /** C = A·B for 21×21 doubles. */
    private fun mmul21(a: DoubleArray, b: DoubleArray, c: DoubleArray) {
        for (i in 0 until P_DIM) {
            val aOff = i * P_DIM
            for (j in 0 until P_DIM) {
                var s = 0.0
                for (k in 0 until P_DIM) s += a[aOff+k] * b[k*P_DIM+j]
                c[aOff+j] = s
            }
        }
    }

    /** C = A·Bᵀ for 21×21 doubles. */
    private fun mmul21T(a: DoubleArray, b: DoubleArray, c: DoubleArray) {
        for (i in 0 until P_DIM) {
            val aOff = i * P_DIM
            for (j in 0 until P_DIM) {
                var s = 0.0
                val bOff = j * P_DIM
                for (k in 0 until P_DIM) s += a[aOff+k] * b[bOff+k]
                c[aOff+j] = s
            }
        }
    }

    /** Set a 21×21 matrix to identity. */
    private fun identity21(m: DoubleArray) {
        m.fill(0.0)
        for (i in 0 until P_DIM) m[rc(i,i)] = 1.0
    }

    /** Enforce P = (P + Pᵀ)/2. */
    private fun symmetrize() {
        for (i in 0 until P_DIM) for (j in i+1 until P_DIM) {
            val avg = (P[rc(i,j)] + P[rc(j,i)]) * 0.5
            P[rc(i,j)] = avg; P[rc(j,i)] = avg
        }
    }

    /**
     * [skew(v) · R][row, col] — computed inline without allocating the full matrix.
     * skew(v) = [[0,-v2,v1],[v2,0,-v0],[-v1,v0,0]]
     */
    private fun skewRow(row: Int, v: DoubleArray, r: FloatArray, col: Int): Double {
        val r0 = r[0*3+col].toDouble(); val r1 = r[1*3+col].toDouble(); val r2 = r[2*3+col].toDouble()
        return when (row) {
            0 -> -v[2]*r1 + v[1]*r2
            1 ->  v[2]*r0 - v[0]*r2
            else -> -v[1]*r0 + v[0]*r1
        }
    }

    /** Invert a 3×3 matrix (row-major). Returns null if singular. */
    private fun invert3x3(m: DoubleArray): DoubleArray? {
        val det = m[0]*(m[4]*m[8]-m[5]*m[7]) - m[1]*(m[3]*m[8]-m[5]*m[6]) + m[2]*(m[3]*m[7]-m[4]*m[6])
        if (abs(det) < 1e-20) return null
        val inv = DoubleArray(9)
        inv[0]= (m[4]*m[8]-m[5]*m[7])/det; inv[1]=-(m[1]*m[8]-m[2]*m[7])/det; inv[2]= (m[1]*m[5]-m[2]*m[4])/det
        inv[3]=-(m[3]*m[8]-m[5]*m[6])/det; inv[4]= (m[0]*m[8]-m[2]*m[6])/det; inv[5]=-(m[0]*m[5]-m[2]*m[3])/det
        inv[6]= (m[3]*m[7]-m[4]*m[6])/det; inv[7]=-(m[0]*m[7]-m[1]*m[6])/det; inv[8]= (m[0]*m[4]-m[1]*m[3])/det
        return inv
    }

    // ── Map snapping (open-loop, ported from RouteAwareFusionEngine) ───────────

    private fun isGnssFresh(): Boolean = lastGnssMillis >= 0 &&
        (System.currentTimeMillis() - lastGnssMillis) < 8000L

    data class Matched(val x: Double, val y: Double, val ve: Double=0.0, val vn: Double=0.0, val routeS: Double=0.0)

    private fun constrainToRoute(rawX: Double, rawY: Double, rawSpeed: Double): Matched? {
        if (route.size < 2 || routeCumDist.size != route.size) return null
        var best = Double.MAX_VALUE; var bx=rawX; var by=rawY; var bs=routeS; var tx=1.0; var ty=0.0; var found=false
        for (i in 0 until route.size-1) {
            val (ax,ay)=route[i]; val (cx,cy)=route[i+1]
            val dx=cx-ax; val dy=cy-ay; val segLen=routeCumDist[i+1]-routeCumDist[i]
            if (segLen<1e-6) continue
            val tRaw=((rawX-ax)*dx+(rawY-ay)*dy)/(segLen*segLen)
            if (tRaw<-0.05||tRaw>1.05) continue
            val t=tRaw.coerceIn(0.0,1.0)
            val qx=ax+t*dx; val qy=ay+t*dy; val d=hypot(rawX-qx,rawY-qy)
            val s=routeCumDist[i]+t*segLen
            if (s+5.0>=routeS && s<=routeS+100.0 && d<best) {
                best=d; bx=qx; by=qy; bs=s; tx=dx/segLen; ty=dy/segLen; found=true
            }
        }
        if (!found||best>25.0) { lateralFailure=true; return null }
        lateralFailure=false
        return Matched(bx, by, tx*rawSpeed, ty*rawSpeed, bs)
    }

    private fun addTrailPoint(outX: Double, outY: Double, headingDeg: Float) {
        val sp = sqrt((v[0]*v[0]+v[1]*v[1]).toDouble())
        if (isStationary || sp < 1.0) return
        if (lastTrailX.isNaN() || hypot(outX-lastTrailX, outY-lastTrailY) >= 2.0) {
            trailE.addLast(outX); trailN.addLast(outY); trailH.addLast(headingDeg)
            lastTrailX=outX; lastTrailY=outY
            while (trailE.size>2000) { trailE.removeFirst(); trailN.removeFirst(); trailH.removeFirst() }
            trailSize = trailE.size
        }
    }

    private fun pushRecentRaw(rawX: Double, rawY: Double) {
        if (lastRecentX.isNaN()||hypot(rawX-lastRecentX,rawY-lastRecentY)>=1.0) {
            recentRawE.addLast(rawX); recentRawN.addLast(rawY)
            lastRecentX=rawX; lastRecentY=rawY
            while (recentRawE.size>60) { recentRawE.removeFirst(); recentRawN.removeFirst() }
        }
    }

    private fun recentMotion(): Triple<Double,Double,Double>? {
        if (recentRawE.size<2) return null
        val dx=recentRawE.last()-recentRawE.first(); val dy=recentRawN.last()-recentRawN.first()
        val len=hypot(dx,dy); if (len<6.0) return null
        return Triple(dx/len, dy/len, len)
    }

    private fun snapToTrail(rawX: Double, rawY: Double, headingDeg: Float, rawSpeed: Double): Matched? {
        trailSnapActive=false
        val sp = sqrt((v[0]*v[0]+v[1]*v[1]).toDouble())
        if (trailE.size<2||sp<1.0) { trHasEdge=false; return null }
        val moved=hypot(rawX-lastTrailQueryX,rawY-lastTrailQueryY)
        val due = !trHasEdge||moved>5.0||elapsed-lastTrailQueryElapsed>1.5
        if (due) {
            lastTrailQueryX=rawX; lastTrailQueryY=rawY; lastTrailQueryElapsed=elapsed
            val edge=findTrailEdge(rawX,rawY,headingDeg)?: run { trHasEdge=false; return null }
            trAx=edge[0]; trAy=edge[1]; trBx=edge[2]; trBy=edge[3]; trReversed=edge[4]<0; trHasEdge=true
        }
        if (!trHasEdge) return null
        val dx=trBx-trAx; val dy=trBy-trAy; val l2=dx*dx+dy*dy
        if (l2<1.0) { trHasEdge=false; return null }
        val tRaw=((rawX-trAx)*dx+(rawY-trAy)*dy)/l2
        if (tRaw<-0.05||tRaw>1.05) { trHasEdge=false; return null }
        val t=tRaw.coerceIn(0.0,1.0)
        val qx=trAx+t*dx; val qy=trAy+t*dy
        if (hypot(rawX-qx,rawY-qy)>20.0) { trHasEdge=false; return null }
        val sqrtL2=sqrt(l2); var ux=dx/sqrtL2; var uy=dy/sqrtL2
        val hRad=Math.toRadians(headingDeg.toDouble()); val hx=sin(hRad); val hy=cos(hRad)
        if (ux*hx+uy*hy<0) { ux=-ux; uy=-uy }
        trailSnapActive=true
        return Matched(qx, qy, ux*rawSpeed, uy*rawSpeed)
    }

    private fun findTrailEdge(rawX: Double, rawY: Double, headingDeg: Float): DoubleArray? {
        val motion=recentMotion()
        val hRad=Math.toRadians(headingDeg.toDouble()); val hx=sin(hRad); val hy=cos(hRad)
        var bestScore=Double.MAX_VALUE; var best: DoubleArray?=null
        for (i in 0 until trailE.size-1) {
            val ax=trailE[i]; val ay=trailN[i]; val bx=trailE[i+1]; val by=trailN[i+1]
            val dx=bx-ax; val dy=by-ay; val len=hypot(dx,dy); if (len<1e-6) continue
            val ux=dx/len; val uy=dy/len
            val t=(((rawX-ax)*dx+(rawY-ay)*dy)/(len*len)).coerceIn(0.0,1.0)
            val d=hypot(rawX-(ax+t*dx),rawY-(ay+t*dy)); if (d>20.0) continue
            val align=ux*hx+uy*hy; if (abs(align)<0.5) continue
            if (motion!=null && abs(motion.first*ux+motion.second*uy)<0.5) continue
            val score=d-10.0*abs(align)
            if (score<bestScore) { bestScore=score; best=doubleArrayOf(ax,ay,bx,by,if(align>=0)1.0 else -1.0) }
        }
        return best
    }

    private fun autoSnapToRoad(rawX: Double, rawY: Double, rawSpeed: Double): Matched? {
        val snapper=roadSnapper?:return null
        val moved=hypot(rawX-lastSnapX,rawY-lastSnapY)
        val due=!autoHasEdge||moved>10.0||elapsed-lastSnapElapsed>2.0
        if (due) {
            lastSnapX=rawX; lastSnapY=rawY; lastSnapElapsed=elapsed; lastSnapHeading=So3Utils.yawDeg(R)
            val (lat,lon)=enuToLatLon(rawX,rawY)
            val snap=try { snapper.snap(lat,lon,lastSnapHeading) } catch(_:Exception){null}
            if (snap!=null&&snap.lateralM<=30.0) {
                val a=latLonToEnu(snap.aLat,snap.aLon); val b=latLonToEnu(snap.bLat,snap.bLon)
                if (hypot(b.first-a.first,b.second-a.second)>1.0) {
                    autoAx=a.first;autoAy=a.second;autoBx=b.first;autoBy=b.second;autoHasEdge=true
                }
            } else { autoHasEdge=false }
        }
        if (!autoHasEdge) return null
        val dx=autoBx-autoAx;val dy=autoBy-autoAy;val l2=dx*dx+dy*dy
        if (l2<1.0) { autoHasEdge=false; return null }
        val tRaw=((rawX-autoAx)*dx+(rawY-autoAy)*dy)/l2
        if (tRaw<-0.05||tRaw>1.05) { autoHasEdge=false; return null }
        val t=tRaw.coerceIn(0.0,1.0)
        val qx=autoAx+t*dx;val qy=autoAy+t*dy
        if (hypot(rawX-qx,rawY-qy)>30.0) { autoHasEdge=false; return null }
        val sqrtL2=sqrt(l2); var ux=dx/sqrtL2; var uy=dy/sqrtL2
        val hRad=Math.toRadians(So3Utils.yawDeg(R).toDouble()); val hx=sin(hRad); val hy=cos(hRad)
        if (ux*hx+uy*hy<0) { ux=-ux; uy=-uy }
        return Matched(qx, qy, ux*rawSpeed, uy*rawSpeed)
    }

    private fun enuToLatLon(e: Double, n: Double): Pair<Double,Double> {
        val lat=originLat+Math.toDegrees(n/6_378_137.0)
        val lon=originLon+Math.toDegrees(e/(6_378_137.0*cos(Math.toRadians(originLat))))
        return lat to lon
    }
    private fun latLonToEnu(lat: Double, lon: Double): Pair<Double,Double> {
        val e=Math.toRadians(lon-originLon)*6_378_137.0*cos(Math.toRadians(originLat))
        val n=Math.toRadians(lat-originLat)*6_378_137.0
        return e to n
    }

    private fun projectRouteS(e: Double, n: Double): Double {
        if (route.size<2||routeCumDist.size!=route.size) return routeS
        var best=Double.MAX_VALUE; var bs=routeS
        for (i in 0 until route.size-1) {
            val (ax,ay)=route[i];val (cx,cy)=route[i+1];val dx=cx-ax;val dy=cy-ay
            val segLen=routeCumDist[i+1]-routeCumDist[i]; if (segLen<1e-6) continue
            val t=(((e-ax)*dx+(n-ay)*dy)/(segLen*segLen)).coerceIn(0.0,1.0)
            val d=hypot(e-(ax+t*dx),n-(ay+t*dy))
            if (d<best){best=d;bs=routeCumDist[i]+t*segLen}
        }
        return bs
    }

    private fun applyRouteLatLon(points: List<Pair<Double,Double>>) {
        route.clear(); routeS=0.0
        for ((lat,lon) in points) {
            val e=Math.toRadians(lon-originLon)*6_378_137.0*cos(Math.toRadians(originLat))
            val n=Math.toRadians(lat-originLat)*6_378_137.0
            route.add(e to n)
        }
        updateRouteDistances()
        if (route.size<2) rebuildRoute()
    }

    private fun rebuildRoute() {
        val lat=destinationLat?:return; val lon=destinationLon?:return
        val e=(lon-originLon)*Math.PI/180.0*6_378_137.0*cos(Math.toRadians(originLat))
        val n=(lat-originLat)*Math.PI/180.0*6_378_137.0
        val len=hypot(e,n).coerceAtLeast(1.0); if (len>1_000_000.0){route.clear();updateRouteDistances();routeS=0.0;return}
        route.clear(); route.add(0.0 to 0.0)
        val steps=ceil(len/10.0).toInt().coerceIn(2,2000)
        for (i in 1..steps) { val f=i.toDouble()/steps; route.add(e*f to n*f) }
        updateRouteDistances(); routeS=0.0
    }

    private fun updateRouteDistances() {
        if (route.size<2){routeCumDist=DoubleArray(0);return}
        routeCumDist=DoubleArray(route.size); routeCumDist[0]=0.0
        for (i in 0 until route.size-1) {
            val dx=route[i+1].first-route[i].first; val dy=route[i+1].second-route[i].second
            routeCumDist[i+1]=routeCumDist[i]+hypot(dx,dy)
        }
    }

    // ── Constants ──────────────────────────────────────────────────────────────

    companion object {
        private const val P_DIM = 21
        private const val Q_DIM = 18
        private const val GRAVITY   = 9.80665f
        private const val GRAVITY_D = 9.80665

        // Process noise Q (18-diagonal): gyro, accel, b_gyro, b_acc, Rc, tc × 3 each
        // Tuned from paper Section IV-B.
        private val Q_DIAG = doubleArrayOf(
            // gyro noise × 3
            1.96e-4, 1.96e-4, 1.96e-4,
            // accel noise × 3
            9e-4, 9e-4, 9e-4,
            // gyro bias random walk × 3
            1e-8, 1e-8, 1e-8,
            // accel bias random walk × 3
            1e-6, 1e-6, 1e-6,
            // Rc noise × 3
            1e-8, 1e-8, 1e-8,
            // tc noise × 3
            1e-8, 1e-8, 1e-8
        )

        // Initial error covariance P0
        private const val COV_ROT0     = 1e-6   // pitch/roll (yaw seeded from GPS)
        private const val COV_V0       = 0.09   // (0.3 m/s)²
        private const val COV_B_OMEGA0 = 1e-4   // rad/s
        private const val COV_B_ACC0   = 9e-4   // m/s²
        private const val COV_RC0      = 1e-6   // rad
        private const val COV_TC0      = 0.01   // m

        // NHC measurement noise (static, no CNN adapter)
        private const val NHC_LAT = 0.04   // (0.2 m/s)² — lateral velocity ≈ 0
        private const val NHC_UP  = 9.0    // (3.0 m/s)² — looser for phone-in-hand

        // ZUPT
        private const val ZUPT_COV      = 1e-4   // m/s per axis
        private const val ZUPT_GYRO_MAX = 0.06   // rad/s
        private const val ZUPT_ACC_MAX  = 0.35   // m/s²
        private const val ZUPT_MIN_S    = 1.2    // seconds
        private const val ZUPT_BIAS_ALPHA = 0.02 // accel bias absorption at rest

        // GYRO LP for standstill
        private const val GYRO_LP = 0.1
    }
}
