package com.sih.idr.navigation

import com.sih.idr.data.GnssMeasurement
import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState
import com.sih.idr.utils.GeoUtils
import kotlin.math.*

/**
 * State-of-the-art vehicle dead reckoning fusion engine:
 * 1. Full 3D attitude rotation: rotates all 3 linear acceleration axes (including Z)
 *    into world ENU frame so tilted phone mounts do not corrupt horizontal acceleration.
 * 2. Fused heading tracking: rotation-vector device yaw (+ auto mount calibration)
 *    + gyro-propagated yaw (continuity across rotation-vector jumps) + filtered
 *    major-velocity-vector heading (slow correction toward the actual path when
 *    moving straight). Rate-limited so jitter never snaps the trail.
 * 3. Zero Velocity Updates (ZUPT): monitors standstill intervals to reset velocity to zero and
 *    cancel accelerometer bias, arresting the primary cause of explosive drift at traffic stops.
 * 4. True non-holonomic vehicle constraints with turn-aware lateral handling: the
 *    2D velocity is decomposed into forward/lateral relative to travel heading;
 *    expected centripetal acceleration (v * yawRate) is subtracted before lateral
 *    integration, lateral velocity is heavily damped (more in turns), and small
 *    residual sideways velocities are dead-banded to zero — this is what stops
 *    turn inertia from flinging the trail sideways.
 * 5. Exact cumulative arc-length route constraint + breadcrumb trail memory +
 *    destination-less road snap, in that priority order. The trail memory lets a
 *    U-turn / backtrack re-snap onto the already-driven path (either direction)
 *    when the road graph has no edge there.
 */
class RouteAwareFusionEngine : DeadReckoningEngine, EngineDebugReporter {
    private var state: NavigationState? = null
    private var lastNanos = -1L
    private var elapsed = 0.0
    private var originLat = 0.0
    private var originLon = 0.0

    // RAW inertial dead-reckoning state (relative meters from seed origin).
    private var x = 0.0
    private var y = 0.0
    var forwardSpeed = 0.0
        private set
    /** Signed lateral (right-positive) velocity after NHC damping (m/s). */
    var lateralVel = 0.0
        private set
    private var ve = 0.0
    private var vn = 0.0

    // ---- Filtered major velocity vector (dominant direction of travel) ----
    private var filtVe = 0.0
    private var filtVn = 0.0
    private var filtInit = false
    /** Slow, stable dominant-path heading, steered only when moving straight. */
    var dominantHeadingDeg = 0f
        private set
    private var dominantInit = false

    // ---- Heading fusion state (degrees clockwise from North) ----
    var vehicleHeadingDeg = 0f
        private set
    /** Gyro-propagated yaw: continuity across rotation-vector jumps. */
    private var gyroHeadingDeg = 0f
    private var gyroHeadingInit = false
    private var lastDeviceYawDeg: Double? = null
    /** Filtered yaw rate, deg/s, +clockwise. */
    var yawRateDegS = 0.0
        private set
    private var lpGyroMag = 0.0

    // Mount alignment: angular offset between device azimuth and vehicle travel heading.
    var mountOffsetDeg = 0.0
        private set
    var mountCalibrated = false
        private set

    // Standstill detection / ZUPT state.
    private var stillDuration = 0.0
    var isStationary = false
        private set
    var zuptCount = 0L
        private set

    // Low-passed linear acceleration in phone frame (IIR).
    private var lpAx = 0.0
    private var lpAy = 0.0
    private var lpAz = 0.0
    private var lpInit = false

    // Online accelerometer bias estimates in world ENU frame.
    private var biasE = 0.0
    private var biasN = 0.0

    // DR_POSITION_FROZEN detector: tracks the output position.
    private var lastOutX = Double.NaN
    private var lastOutY = Double.NaN
    private var lastMoveElapsed = 0.0
    private var freezeLogged = false
    private var lastAccMag = Float.NaN

    // GNSS gating state.
    private var lastGnssMillis = -1L
    private var gnssRejectStreak = 0
    var gnssRejects = 0
        private set

    /**
     * Destination-less map matching: snaps onto the nearest road in the
     * direction of travel when no planned route exists. Wired by the app
     * layer ([RoadSnapProvider]); null = free inertial coast.
     */
    var roadSnapper: RoadSnapProvider? = null
    private var autoAx = 0.0
    private var autoAy = 0.0
    private var autoBx = 0.0
    private var autoBy = 0.0
    private var autoHasEdge = false
    private var autoSnapped = false
    private var lastSnapX = 0.0
    private var lastSnapY = 0.0
    private var lastSnapElapsed = -1.0
    private var lastSnapHeading = 0f

    /** True while laterally pinned to an auto-discovered road segment. */
    var autoSnapActive = false
        private set

    // ---- Breadcrumb trail memory (own driven path) ----
    private val trailE = ArrayDeque<Double>()
    private val trailN = ArrayDeque<Double>()
    private val trailH = ArrayDeque<Float>()
    private var lastTrailX = Double.NaN
    private var lastTrailY = Double.NaN
    // Cached trail edge for cheap per-sample projection.
    private var trAx = 0.0
    private var trAy = 0.0
    private var trBx = 0.0
    private var trBy = 0.0
    private var trHasEdge = false
    private var trReversed = false
    private var lastTrailQueryX = 0.0
    private var lastTrailQueryY = 0.0
    private var lastTrailQueryElapsed = -1.0
    /** True while pinned to the remembered driven path. */
    var trailSnapActive = false
        private set
    /** Which constraint produced the current output (for logging/UI). */
    var snapMode = "FREE"
        private set
    val trailSize: Int get() = trailE.size
    // Recent raw-position buffer for the resemblance gate (decimated ~5 Hz).
    private val recentRawE = ArrayDeque<Double>()
    private val recentRawN = ArrayDeque<Double>()
    private var lastRecentX = Double.NaN
    private var lastRecentY = Double.NaN

    override var lastDebug: com.sih.idr.navigation.pdr.PdrDebug? = null
        private set

    private val stepCounter = com.sih.idr.navigation.pdr.DynamicStepCounter(1.0)
    var stepCount = 0
        private set

    private var route: MutableList<Pair<Double, Double>> = mutableListOf()
    private var routeCumDist: DoubleArray = DoubleArray(0)
    private var destinationLat: Double? = null
    private var destinationLon: Double? = null
    private var pendingRouteLatLon: List<Pair<Double, Double>>? = null
    private var routeS = 0.0

    var status: String = "waiting for GNSS"
        private set
    var gpsAvailable = false
        private set
    var speedFailure = false
        private set
    var lateralFailure = false
        private set

    override val currentState: NavigationState? get() = state

    fun setDestination(latitude: Double, longitude: Double) {
        destinationLat = latitude
        destinationLon = longitude
        if (state == null) return
        rebuildRoute()
    }

    /** Supply a road-following route as latitude/longitude points. */
    fun setRouteLatLon(points: List<Pair<Double, Double>>) {
        pendingRouteLatLon = points.toList()
        if (state == null) return
        applyRouteLatLon(points)
    }

    private fun applyRouteLatLon(points: List<Pair<Double, Double>>) {
        route.clear()
        routeS = 0.0
        for ((lat, lon) in points) {
            val e = Math.toRadians(lon - originLon) * 6378137.0 * cos(Math.toRadians(originLat))
            val n = Math.toRadians(lat - originLat) * 6378137.0
            route.add(e to n)
        }
        updateRouteDistances()
        if (route.size < 2) rebuildRoute()
    }

    private fun rebuildRoute() {
        val latitude = destinationLat ?: return
        val longitude = destinationLon ?: return
        val e = (longitude - originLon) * PI / 180.0 * 6378137.0 * cos(Math.toRadians(originLat))
        val n = (latitude - originLat) * PI / 180.0 * 6378137.0
        val len = hypot(e, n).coerceAtLeast(1.0)
        if (len > REBUILD_MAX_LEN_M) {
            route.clear()
            updateRouteDistances()
            routeS = 0.0
            return
        }
        route.clear()
        route.add(0.0 to 0.0)
        val steps = ceil(len / 10.0).toInt().coerceIn(2, 2000)
        for (i in 1..steps) {
            val f = i.toDouble() / steps
            route.add(e * f to n * f)
        }
        updateRouteDistances()
        routeS = 0.0
    }

    private fun updateRouteDistances() {
        if (route.size < 2) {
            routeCumDist = DoubleArray(0)
            return
        }
        routeCumDist = DoubleArray(route.size)
        routeCumDist[0] = 0.0
        for (i in 0 until route.size - 1) {
            val dx = route[i + 1].first - route[i].first
            val dy = route[i + 1].second - route[i].second
            routeCumDist[i + 1] = routeCumDist[i] + hypot(dx, dy)
        }
    }

    fun clearRoute() {
        route.clear()
        updateRouteDistances()
        routeS = 0.0
        destinationLat = null
        destinationLon = null
    }

    fun routeLatLon(): List<Pair<Double, Double>> =
        route.map { GeoUtils.moveEnu(originLat, originLon, it.second, it.first) }

    override fun initialize(initialState: NavigationState) {
        reset()
        state = initialState
        originLat = initialState.latitude
        originLon = initialState.longitude
        forwardSpeed = initialState.speed.toDouble()
        vehicleHeadingDeg = initialState.headingDeg
        gyroHeadingDeg = initialState.headingDeg
        gyroHeadingInit = true
        dominantHeadingDeg = initialState.headingDeg
        dominantInit = true
        val hRad = Math.toRadians(initialState.headingDeg.toDouble())
        ve = forwardSpeed * sin(hRad)
        vn = forwardSpeed * cos(hRad)
        filtVe = ve
        filtVn = vn
        filtInit = true
        status = "GNSS anchor acquired"
        pendingRouteLatLon?.let { applyRouteLatLon(it) } ?: rebuildRoute()
    }

    override fun processImu(sample: ImuSample): NavigationState {
        val old = state ?: return NavigationState(sample.timestampNanos, 0.0, 0.0)
        val dt = if (lastNanos < 0) 0.01 else ((sample.timestampNanos - lastNanos) / 1e9).coerceIn(0.001, 0.2)
        lastNanos = sample.timestampNanos
        elapsed += dt

        // 1. Extract complete 3D linear acceleration (phone frame)
        var ax = sample.linearX?.toDouble()
        var ay = sample.linearY?.toDouble()
        var az = sample.linearZ?.toDouble()

        val qx = sample.quatX; val qy = sample.quatY; val qz = sample.quatZ; val qw = sample.quatW
        val rotM = if (qx != null && qy != null && qz != null && qw != null) {
            CoordinateTransformer.rotationMatrixFromQuaternion(qx, qy, qz, qw)
        } else null

        if (ax == null || ay == null || az == null) {
            if (rotM != null) {
                val gPhone = FloatArray(3)
                CoordinateTransformer.gravityInPhoneFrame(rotM, gPhone)
                ax = (sample.accelX - gPhone[0]).toDouble()
                ay = (sample.accelY - gPhone[1]).toDouble()
                az = (sample.accelZ - gPhone[2]).toDouble()
            } else {
                ax = sample.accelX.toDouble()
                ay = sample.accelY.toDouble()
                az = (sample.accelZ - 9.80665f).toDouble()
            }
        }

        lastAccMag = sqrt(ax * ax + ay * ay + az * az).toFloat()

        // 2. Low-pass filter linear acceleration across all 3 axes (IIR ~4 Hz)
        if (!lpInit) {
            lpAx = ax; lpAy = ay; lpAz = az
            lpInit = true
        }
        lpAx += ACC_LP_ALPHA * (ax - lpAx)
        lpAy += ACC_LP_ALPHA * (ay - lpAy)
        lpAz += ACC_LP_ALPHA * (az - lpAz)

        // Clamp shock spikes
        var fax = lpAx; var fay = lpAy; var faz = lpAz
        val aMag = sqrt(fax * fax + fay * fay + faz * faz)
        if (aMag > ACC_MAX && aMag > 0.0) {
            val s = ACC_MAX / aMag
            fax *= s; fay *= s; faz *= s
        }

        // 3. Heading fusion: device yaw + gyro propagation + velocity-vector correction.
        // 3a. Device yaw from the rotation vector (absolute, mount-offset corrected).
        var deviceYawDeg: Double? = null
        if (rotM != null) {
            val ypr = CoordinateTransformer.yawPitchRollDeg(rotM)
            deviceYawDeg = ypr[0].toDouble()
        }
        // 3b. Filtered gyro magnitude (turn-energy gate when rotM is absent).
        val gyroMag = sqrt(
            sample.gyroX * sample.gyroX + sample.gyroY * sample.gyroY + sample.gyroZ * sample.gyroZ
        ).toDouble()
        lpGyroMag += GYRO_LP_ALPHA * (gyroMag - lpGyroMag)
        // 3c. Yaw rate from rotation-vector deltas (OS gyro-fused, mount-independent
        // in the delta domain); decays to zero when the attitude is unavailable.
        if (deviceYawDeg != null) {
            if (!gyroHeadingInit) {
                gyroHeadingDeg = GeoUtils.normalizeHeading((deviceYawDeg + mountOffsetDeg).toFloat())
                gyroHeadingInit = true
                lastDeviceYawDeg = deviceYawDeg
                yawRateDegS = 0.0
            } else {
                val prev = lastDeviceYawDeg
                if (prev != null && dt > 0.0) {
                    val delta = GeoUtils.angleDiffDeg(deviceYawDeg, prev)
                    val instRate = delta / dt
                    // Clamp unphysical jumps (> 180 deg/s) before they poison the filter.
                    val clamped = instRate.coerceIn(-180.0, 180.0)
                    yawRateDegS += YAW_RATE_ALPHA * (clamped - yawRateDegS)
                    // Propagate the gyro heading with the same (clamped) delta so it
                    // rides through rotation-vector dropouts without jumping.
                    val step = clamped * dt
                    gyroHeadingDeg = GeoUtils.normalizeHeading((gyroHeadingDeg + step).toFloat())
                }
                lastDeviceYawDeg = deviceYawDeg
            }
        } else {
            yawRateDegS *= (1.0 - YAW_RATE_ALPHA)
            if (lpGyroMag > TURN_GYRO_MAG_RAD_S) {
                // Attitude unknown but the gyro sees rotation: hold heading, flag turn
                // energy so lateral integration stays suppressed (see §6).
                yawRateDegS = yawRateDegS.coerceIn(-40.0, 40.0)
            }
        }
        // 3d. Base heading: absolute device yaw when available, else gyro coast.
        var fusedHeading = if (deviceYawDeg != null) {
            GeoUtils.normalizeHeading((deviceYawDeg + mountOffsetDeg).toFloat())
        } else {
            gyroHeadingDeg
        }
        // Blend the gyro-propagated yaw toward the absolute yaw so neither can run
        // away: gyro carries high-frequency turns, device yaw anchors drift.
        if (deviceYawDeg != null && gyroHeadingInit) {
            val drift = GeoUtils.angleDiffDeg(fusedHeading.toDouble(), gyroHeadingDeg.toDouble())
            gyroHeadingDeg = GeoUtils.normalizeHeading((gyroHeadingDeg + GYRO_ANCHOR_W * drift).toFloat())
        }

        // 4. Transform 3D linear acceleration to world frame (ENU)
        var aEast = 0.0
        var aNorth = 0.0
        if (rotM != null) {
            val aPhone = floatArrayOf(fax.toFloat(), fay.toFloat(), faz.toFloat())
            val aWorld = FloatArray(3)
            CoordinateTransformer.transform(aPhone, rotM, aWorld)
            aEast = aWorld[0].toDouble()
            aNorth = aWorld[1].toDouble()
        } else {
            val yawRadTmp = Math.toRadians(fusedHeading.toDouble())
            aEast = fax * sin(yawRadTmp) + fay * cos(yawRadTmp)
            aNorth = -fax * cos(yawRadTmp) + fay * sin(yawRadTmp)
        }

        // 5. Motion Classification: Step Detection & Standstill (ZUPT)
        val rawAcc = sqrt(sample.accelX * sample.accelX + sample.accelY * sample.accelY + sample.accelZ * sample.accelZ).toDouble()
        val isStep = stepCounter.findStep(rawAcc)
        if (isStep) stepCount++

        val gyroRate = sqrt(sample.gyroX * sample.gyroX + sample.gyroY * sample.gyroY + sample.gyroZ * sample.gyroZ)
        val isQuiet = gyroRate < STANDSTILL_GYRO_MAX && aMag < STANDSTILL_ACC_MAX && !isStep
        if (isQuiet) {
            stillDuration += dt
        } else {
            stillDuration = 0.0
        }

        // Standstill engages after sustained quiescence AND speed is low or braking has stopped vehicle
        isStationary = stillDuration >= STANDSTILL_MIN_TIME_S && (forwardSpeed < 1.2 || stillDuration >= 2.5)

        // Decode current heading unit vectors for the NHC decomposition.
        var yawRad = Math.toRadians(fusedHeading.toDouble())
        var sinH = sin(yawRad)
        var cosH = cos(yawRad)

        if (isStationary) {
            // ZUPT active: zero velocity and calibrate zero-g bias
            forwardSpeed = 0.0
            lateralVel = 0.0
            ve = 0.0
            vn = 0.0
            filtVe = 0.0
            filtVn = 0.0
            biasE += 0.05 * (aEast - biasE)
            biasN += 0.05 * (aNorth - biasN)
            zuptCount++
            status = "Stationary (ZUPT active)"
            vehicleHeadingDeg = fusedHeading
        } else {
            // 6. True NHC: integrate 2D velocity, then split forward/lateral and
            //    damp the lateral channel with turn-aware compensation.
            if (isStep) {
                // Pedestrian step impulse (~1.25 m/s cadence)
                val vFwdNow = ve * sinH + vn * cosH
                if (vFwdNow < 1.25) {
                    val boost = 1.25 - vFwdNow
                    ve += sinH * boost
                    vn += cosH * boost
                }
            }

            val aEffEast = aEast - biasE
            val aEffNorth = aNorth - biasN

            // Raw strapdown integration first (keeps the filter honest)…
            var vE = ve + aEffEast * dt
            var vN = vn + aEffNorth * dt

            // …then decompose relative to travel heading.
            var vFwd = vE * sinH + vN * cosH
            var vLat = vE * cosH - vN * sinH // right-positive
            val aFwdRaw = (aEffEast * sinH + aEffNorth * cosH).coerceIn(-ACC_BRAKE_MAX, ACC_FWD_MAX)
            val aLatRaw = aEffEast * cosH - aEffNorth * sinH

            // Turn compensation: the lateral accelerometer mostly sees centripetal
            // acceleration (v * yawRate) in a turn — that is NOT a lane change.
            // Subtract the expected value and only integrate the residual.
            val yawRateRad = Math.toRadians(yawRateDegS)
            val turning = abs(yawRateDegS) > TURN_YAW_RATE_DEG_S || lpGyroMag > TURN_GYRO_MAG_RAD_S
            val aLatExpected = vFwd * yawRateRad
            var aLatResidual = aLatRaw - aLatExpected
            if (turning) aLatResidual *= TURN_LAT_SUPPRESS
            // Rebuild lateral velocity from the damped prior + suppressed residual
            // instead of the raw strapdown value, so turn inertia cannot fling it.
            val latDamp = LAT_BASE_DAMP + abs(yawRateRad) * LAT_TURN_GAIN
            vLat = lateralVel * exp(-latDamp * dt) + aLatResidual * dt
            // Deadband: ignore slight sideways creep while going straight — the car
            // is not sliding, the MEMS is.
            if (!turning && abs(vLat) < LAT_DEADBAND_MPS) vLat = 0.0
            vLat = vLat.coerceIn(-LAT_MAX_MPS, LAT_MAX_MPS)

            // Longitudinal channel: the strapdown mix above already carries aFwd*dt;
            // here we only enforce bounds + coast drag (no double integration).
            if (abs(aFwdRaw) < 0.15 && !isStep) {
                vFwd *= (1.0 - COAST_DRAG_COEFF * dt)
            }
            vFwd = vFwd.coerceIn(0.0, V_MAX)

            speedFailure = !vFwd.isFinite() || vFwd > V_MAX
            if (!vFwd.isFinite()) {
                vFwd = 0.0
                status = "speed failure: bounded"
            } else if (vFwd >= V_MAX) {
                vFwd = V_MAX
                status = "speed clamped: bound"
            }

            forwardSpeed = vFwd
            lateralVel = vLat
            ve = sinH * vFwd + cosH * vLat
            vn = cosH * vFwd - sinH * vLat

            // 6b. Major velocity-vector filter (EMA, ~0.4 s) + dominant-path latch.
            val velAlpha = (dt / (VEL_FILTER_TAU_S + dt)).coerceIn(0.0, 1.0)
            if (!filtInit) {
                filtVe = ve; filtVn = vn; filtInit = true
            } else {
                filtVe += velAlpha * (ve - filtVe)
                filtVn += velAlpha * (vn - filtVn)
            }
            val filtSpeed = hypot(filtVe, filtVn)
            if (filtSpeed > VEL_HEADING_MIN_SPEED) {
                val velHeading = GeoUtils.normalizeHeading(
                    Math.toDegrees(atan2(filtVe, filtVn)).toFloat()
                )
                if (!dominantInit) {
                    dominantHeadingDeg = velHeading
                    dominantInit = true
                } else if (!turning && filtSpeed > DOMINANT_MIN_SPEED) {
                    // Steer the dominant vector toward the actual path of travel —
                    // slowly, so a single noisy fix never yanks the trail.
                    val domAlpha = (dt / (DOMINANT_TAU_S + dt))
                    val d = GeoUtils.angleDiffDeg(velHeading.toDouble(), dominantHeadingDeg.toDouble())
                    dominantHeadingDeg = GeoUtils.normalizeHeading(
                        (dominantHeadingDeg + (d * domAlpha)).toFloat()
                    )
                }
                // While moving straight, gently pull travel heading toward the
                // dominant vector: this is the "adjust the vector majorly toward
                // the actual path" correction.
                if (!turning && filtSpeed > DOMINANT_MIN_SPEED) {
                    val pullAlpha = (dt / (HEADING_PULL_TAU_S + dt))
                    val dh = GeoUtils.angleDiffDeg(dominantHeadingDeg.toDouble(), fusedHeading.toDouble())
                    val pulled = fusedHeading.toDouble() + dh * pullAlpha
                    fusedHeading = GeoUtils.normalizeHeading(pulled.toFloat())
                }
            }
            // 6c. Heading rate limiter: a car cannot spin faster than this.
            val prevH = vehicleHeadingDeg.toDouble()
            val rawDelta = GeoUtils.angleDiffDeg(fusedHeading.toDouble(), if (elapsed <= dt + 1e-9) fusedHeading.toDouble() else prevH)
            val maxStep = HEADING_MAX_RATE_DEG_S * dt
            val limited = rawDelta.coerceIn(-maxStep, maxStep)
            fusedHeading = GeoUtils.normalizeHeading(
                ((if (elapsed <= dt + 1e-9) fusedHeading.toDouble() else prevH) + limited).toFloat()
            )
            vehicleHeadingDeg = fusedHeading
            yawRad = Math.toRadians(vehicleHeadingDeg.toDouble())
            sinH = sin(yawRad)
            cosH = cos(yawRad)
            // Re-express the velocity along the (possibly pulled) heading so the
            // position step follows the corrected direction.
            ve = sinH * forwardSpeed + cosH * lateralVel
            vn = cosH * forwardSpeed - sinH * lateralVel

            val fresh = isGnssFresh()
            gpsAvailable = fresh
            if (!fresh) {
                status = if (autoSnapActive) "GPS OFF: inertial + road snap"
                else if (trailSnapActive) "GPS OFF: inertial + trail memory"
                else if (route.size >= 2 && !lateralFailure) "GPS OFF: inertial + route"
                else if (isStep) "GPS OFF: pedestrian DR (walking)"
                else "GPS OFF: inertial dead reckoning"
            }
        }

        // 7. Advance RAW position
        x += ve * dt
        y += vn * dt
        pushRecentRaw(x, y)

        // 8. Map Matching / Snapping on a Copy (priority: route > trail > auto)
        var outX = x; var outY = y; var outVe = ve; var outVn = vn
        snapMode = if (isStationary) "ZUPT" else "FREE"
        val routeMatch = constrainToRoute(x, y, forwardSpeed)
        if (routeMatch != null) {
            outX = routeMatch.x; outY = routeMatch.y
            outVe = routeMatch.ve; outVn = routeMatch.vn
            routeS = routeMatch.routeS
            snapMode = "ROUTE"
            trailSnapActive = false
            autoSnapActive = autoSnapped
        } else {
            val trailMatch = snapToTrail(x, y, vehicleHeadingDeg, forwardSpeed)
            if (trailMatch != null) {
                outX = trailMatch.x; outY = trailMatch.y
                outVe = trailMatch.ve; outVn = trailMatch.vn
                snapMode = if (trReversed) "TRAIL-REV" else "TRAIL"
            } else {
                val snap = autoSnapToRoad(x, y, forwardSpeed)
                if (snap != null) {
                    outX = snap.x; outY = snap.y
                    outVe = snap.ve; outVn = snap.vn
                    snapMode = "AUTO"
                }
            }
        }

        // Remember the driven path (decimated): future backtracks snap to this.
        addTrailPoint(outX, outY, vehicleHeadingDeg)

        val outHeadingDeg = if (hypot(outVe, outVn) > 0.5) {
            GeoUtils.normalizeHeading(Math.toDegrees(atan2(outVe, outVn)).toFloat())
        } else {
            vehicleHeadingDeg
        }

        val ll = GeoUtils.moveEnu(originLat, originLon, outY, outX)
        val next = old.copy(
            timestampNanos = sample.timestampNanos,
            latitude = ll.first,
            longitude = ll.second,
            velEast = outVe.toFloat(),
            velNorth = outVn.toFloat(),
            headingDeg = outHeadingDeg,
            confidenceMeters = (2.0 + elapsed * if (gpsAvailable) 0.05 else 0.8).toFloat()
        )
        state = next

        checkFrozen(outX, outY)

        // Engine debug reporter snapshot: EVERY field populated on EVERY sample so
        // nav/pdr/imu rows stay consistent and joinable on timestamp_nanos.
        val deviceRad = if (deviceYawDeg != null) Math.toRadians(deviceYawDeg).toFloat() else Float.NaN
        lastDebug = com.sih.idr.navigation.pdr.PdrDebug(
            timestampNanos = sample.timestampNanos,
            accMagnitude = lastAccMag,
            stepDetected = isStep,
            strideLength = (forwardSpeed * dt).toFloat(),
            gyroHeadingRad = Math.toRadians(gyroHeadingDeg.toDouble()).toFloat(),
            magHeadingRad = deviceRad,
            fusedHeadingRad = Math.toRadians(outHeadingDeg.toDouble()).toFloat(),
            xEast = outX.toFloat(),
            yNorth = outY.toFloat(),
            latitude = ll.first,
            longitude = ll.second,
            forwardSpeedMps = forwardSpeed.toFloat(),
            yawRateDegS = yawRateDegS.toFloat(),
            lateralVelMps = lateralVel.toFloat(),
            snapMode = snapMode
        )

        return next
    }

    override fun onGnssMeasurement(measurement: GnssMeasurement) {
        val g = measurement.sample
        val acc = (g.accuracy ?: 5f).toDouble().coerceAtLeast(1.0)
        if (acc > 30.0) return

        val e = Math.toRadians(g.longitude - originLon) * 6378137.0 * cos(Math.toRadians(originLat))
        val n = Math.toRadians(g.latitude - originLat) * 6378137.0

        val jump = hypot(e - x, n - y)
        val nowMs = g.timestampMillis
        val dtGps = if (lastGnssMillis < 0) 0.0
        else ((nowMs - lastGnssMillis) / 1000.0).coerceIn(0.0, 3600.0)

        val gate = GNSS_GATE_K * acc + min(forwardSpeed, GNSS_GATE_SPEED_CAP) * dtGps + GNSS_GATE_MARGIN
        val forced = gnssRejectStreak >= GNSS_REJECT_STREAK_ACCEPT
        if (!forced && lastGnssMillis >= 0 && dtGps >= 0.0 && jump > gate) {
            gnssRejectStreak++
            gnssRejects++
            status = "GNSS glitch rejected (%.0fm jump, gate %.0fm)".format(jump, gate)
            return
        }

        gnssRejectStreak = 0
        lastGnssMillis = nowMs
        gpsAvailable = true

        // Blend toward the fix proportional to accuracy
        val k = (GNSS_BLEND_REF / acc).coerceIn(GNSS_BLEND_MIN, 1.0)
        x += k * (e - x)
        y += k * (n - y)

        routeS = projectRouteS(x, y)

        // Calibrate forward speed and mount alignment
        g.speed?.let { spd ->
            if (spd.isFinite() && spd in 0f..GNSS_FIX_SPEED_MAX) {
                if (spd > GNSS_BEARING_MIN_SPEED && g.bearing != null) {
                    val bearingDeg = g.bearing.toDouble()
                    val currentRawYaw = vehicleHeadingDeg.toDouble() - mountOffsetDeg
                    val targetOffset = GeoUtils.angleDiffDeg(bearingDeg, currentRawYaw)
                    if (!mountCalibrated) {
                        mountOffsetDeg = targetOffset
                        mountCalibrated = true
                    } else {
                        mountOffsetDeg += 0.08 * GeoUtils.angleDiffDeg(targetOffset, mountOffsetDeg)
                    }
                    forwardSpeed = spd.toDouble()
                    lateralVel = 0.0
                    val hRad = Math.toRadians(bearingDeg)
                    ve = forwardSpeed * sin(hRad)
                    vn = forwardSpeed * cos(hRad)
                    filtVe = ve
                    filtVn = vn
                    filtInit = true
                    vehicleHeadingDeg = GeoUtils.normalizeHeading(bearingDeg.toFloat())
                    gyroHeadingDeg = vehicleHeadingDeg
                    dominantHeadingDeg = vehicleHeadingDeg
                    dominantInit = true
                } else if (spd < 0.5f) {
                    forwardSpeed = 0.0
                    lateralVel = 0.0
                    ve = 0.0; vn = 0.0
                }
            }
        }

        state = state?.copy(
            latitude = if (k >= 1.0) g.latitude else originLat + Math.toDegrees(y / 6378137.0),
            longitude = if (k >= 1.0) g.longitude else originLon + Math.toDegrees(x / (6378137.0 * cos(Math.toRadians(originLat)))),
            velEast = ve.toFloat(),
            velNorth = vn.toFloat(),
            headingDeg = vehicleHeadingDeg,
            confidenceMeters = acc.toFloat()
        )

        stillDuration = 0.0
        status = if (route.size >= 2) "GNSS fused: route constrained" else "GNSS fused: inertial calibrated"
    }

    internal data class Matched(val x: Double, val y: Double, val ve: Double, val vn: Double, val routeS: Double = 0.0)

    private fun isGnssFresh(nowMs: Long = System.currentTimeMillis()): Boolean {
        if (lastGnssMillis < 0) return false
        return (nowMs - lastGnssMillis) < 8000L
    }

    private fun checkFrozen(outX: Double, outY: Double) {
        if (isStationary) {
            lastMoveElapsed = elapsed
            freezeLogged = false
            return
        }
        if (lastOutX.isNaN() || lastOutY.isNaN()) {
            lastOutX = outX; lastOutY = outY; lastMoveElapsed = elapsed; freezeLogged = false
            return
        }
        if (hypot(outX - lastOutX, outY - lastOutY) > 0.5) {
            lastOutX = outX; lastOutY = outY; lastMoveElapsed = elapsed; freezeLogged = false
            return
        }
        val stagnantS = elapsed - lastMoveElapsed
        if (!freezeLogged && stagnantS > 30.0 && !isGnssFresh() && forwardSpeed > 1.0) {
            freezeLogged = true
            try {
                android.util.Log.w(
                    "RouteAwareFusionEngine",
                    "DR_POSITION_FROZEN gpsUnavailable speed=%.2f stagnant=%.0fs out=(%.1f,%.1f) raw=(%.1f,%.1f)".format(
                        forwardSpeed, stagnantS, outX, outY, x, y
                    )
                )
            } catch (_: Throwable) {
                println("[RouteAwareFusionEngine] DR_POSITION_FROZEN speed=$forwardSpeed stagnant=$stagnantS")
            }
        }
    }

    internal fun constrainToRoute(rawX: Double, rawY: Double, rawSpeed: Double = forwardSpeed): Matched? {
        if (route.size < 2 || routeCumDist.size != route.size) return null
        var best = Double.POSITIVE_INFINITY
        var bx = rawX; var by = rawY; var bs = routeS
        var tx = 1.0; var ty = 0.0
        var found = false

        for (i in 0 until route.size - 1) {
            val (ax, ay) = route[i]
            val (cx, cy) = route[i + 1]
            val dx = cx - ax; val dy = cy - ay
            val segLen = routeCumDist[i + 1] - routeCumDist[i]
            if (segLen < 1e-6) continue
            // Fraction along segment line without clamping
            val tRaw = ((rawX - ax) * dx + (rawY - ay) * dy) / (segLen * segLen)
            // Only consider it on-segment if within bounds (-5% to 105%)
            if (tRaw < -0.05 || tRaw > 1.05) continue

            val t = tRaw.coerceIn(0.0, 1.0)
            val qx = ax + t * dx
            val qy = ay + t * dy
            val d = hypot(rawX - qx, rawY - qy)
            val s = routeCumDist[i] + t * segLen

            if (s + 5.0 >= routeS && s <= routeS + ROUTE_LOOKAHEAD_M && d < best) {
                best = d
                bx = qx
                by = qy
                bs = s
                tx = dx / segLen
                ty = dy / segLen
                found = true
            }
        }
        if (!found || best > 25.0) {
            lateralFailure = true
            return null
        }
        lateralFailure = false
        val mve = tx * rawSpeed
        val mvn = ty * rawSpeed
        return Matched(bx, by, mve, mvn, bs)
    }

    // ---------------------------------------------------------------- trail memory
    private fun addTrailPoint(outX: Double, outY: Double, headingDeg: Float) {
        if (isStationary || forwardSpeed < TRAIL_MIN_SPEED) return
        if (lastTrailX.isNaN() || hypot(outX - lastTrailX, outY - lastTrailY) >= TRAIL_MIN_DIST_M) {
            trailE.addLast(outX); trailN.addLast(outY); trailH.addLast(headingDeg)
            lastTrailX = outX; lastTrailY = outY
            while (trailE.size > TRAIL_MAX_POINTS) {
                trailE.removeFirst(); trailN.removeFirst(); trailH.removeFirst()
            }
        }
    }

    private fun pushRecentRaw(rawX: Double, rawY: Double) {
        if (lastRecentX.isNaN() || hypot(rawX - lastRecentX, rawY - lastRecentY) >= RECENT_MIN_DIST_M) {
            recentRawE.addLast(rawX); recentRawN.addLast(rawY)
            lastRecentX = rawX; lastRecentY = rawY
            while (recentRawE.size > RECENT_MAX_POINTS) {
                recentRawE.removeFirst(); recentRawN.removeFirst()
            }
        }
    }

    /** Recent raw motion vector (for the resemblance gate): null when too short. */
    private fun recentMotion(): Triple<Double, Double, Double>? {
        if (recentRawE.size < 2) return null
        val e0 = recentRawE.first(); val n0 = recentRawN.first()
        val e1 = recentRawE.last(); val n1 = recentRawN.last()
        val dx = e1 - e0; val dy = n1 - n0
        val len = hypot(dx, dy)
        if (len < TRAIL_RESEMBLE_MIN_LEN_M) return null
        return Triple(dx / len, dy / len, len)
    }

    /**
     * Snap onto the remembered driven path when the fresh trail resembles it.
     * Accepts BOTH travel directions (forward + reverse/backtrack): a U-turn or
     * return leg re-pins to the old centerline instead of starting a parallel
     * ghost trail. Throttled with a cached edge like the road-graph snap.
     */
    internal fun snapToTrail(rawX: Double, rawY: Double, headingDeg: Float, rawSpeed: Double): Matched? {
        trailSnapActive = false
        if (trailE.size < 2 || rawSpeed < TRAIL_MIN_SPEED) {
            trHasEdge = false
            return null
        }
        val moved = hypot(rawX - lastTrailQueryX, rawY - lastTrailQueryY)
        val due = !trHasEdge || moved > TRAIL_REQUERY_M || elapsed - lastTrailQueryElapsed > TRAIL_REQUERY_S
        if (due) {
            lastTrailQueryX = rawX; lastTrailQueryY = rawY; lastTrailQueryElapsed = elapsed
            val edge = findTrailEdge(rawX, rawY, headingDeg) ?: run {
                trHasEdge = false
                return null
            }
            trAx = edge[0]; trAy = edge[1]; trBx = edge[2]; trBy = edge[3]
            trReversed = edge[4] < 0
            trHasEdge = true
        }
        if (!trHasEdge) return null
        val dx = trBx - trAx; val dy = trBy - trAy
        val l2 = dx * dx + dy * dy
        if (l2 < 1.0) {
            trHasEdge = false
            return null
        }
        val tRaw = ((rawX - trAx) * dx + (rawY - trAy) * dy) / l2
        if (tRaw < -0.05 || tRaw > 1.05) {
            trHasEdge = false
            return null
        }
        val t = tRaw.coerceIn(0.0, 1.0)
        val qx = trAx + t * dx; val qy = trAy + t * dy
        if (hypot(rawX - qx, rawY - qy) > TRAIL_SNAP_RADIUS_M) {
            trHasEdge = false
            return null
        }
        var ux = dx / sqrt(l2); var uy = dy / sqrt(l2)
        val hRad = Math.toRadians(headingDeg.toDouble())
        val hx = sin(hRad); val hy = cos(hRad)
        if (ux * hx + uy * hy < 0) {
            ux = -ux; uy = -uy
        }
        trailSnapActive = true
        return Matched(qx, qy, ux * rawSpeed, uy * rawSpeed)
    }

    /**
     * Finds the trail segment that best explains the current motion.
     * Returns [ax, ay, bx, by, dirSign] or null. Requires BOTH proximity AND
     * resemblance: the recent raw motion must parallel the old segment (same or
     * opposite direction) so jitter near an old parking spot cannot pin the fix.
     */
    private fun findTrailEdge(rawX: Double, rawY: Double, headingDeg: Float): DoubleArray? {
        val motion = recentMotion()
        val hRad = Math.toRadians(headingDeg.toDouble())
        val hx = sin(hRad); val hy = cos(hRad)
        var bestScore = Double.POSITIVE_INFINITY
        var best: DoubleArray? = null
        // Stride 1 over ≤2000 points is fine at the throttled requery rate.
        for (i in 0 until trailE.size - 1) {
            val ax = trailE[i]; val ay = trailN[i]
            val bx = trailE[i + 1]; val by = trailN[i + 1]
            val dx = bx - ax; val dy = by - ay
            val len = hypot(dx, dy)
            if (len < 1e-6) continue
            val ux = dx / len; val uy = dy / len
            val t = (((rawX - ax) * dx + (rawY - ay) * dy) / (len * len)).coerceIn(0.0, 1.0)
            val qx = ax + t * dx; val qy = ay + t * dy
            val d = hypot(rawX - qx, rawY - qy)
            if (d > TRAIL_SNAP_RADIUS_M) continue
            val align = ux * hx + uy * hy // +1 same dir, -1 reverse
            // Resemblance gate: heading must match the old segment either way.
            // Reverse/backtrack legs match with align ≈ -1 — that is the point.
            if (abs(align) < TRAIL_MIN_ALIGN) continue
            if (motion != null) {
                val mAlign = motion.first * ux + motion.second * uy
                if (abs(mAlign) < TRAIL_MIN_ALIGN) continue
            }
            // Prefer close + aligned; small bonus for same-direction continuity.
            val score = d - TRAIL_ALIGN_WEIGHT_M * abs(align)
            if (score < bestScore) {
                bestScore = score
                // Don't look further back than a short window behind the query:
                // prevents teleporting to a parallel old lap across town.
                best = doubleArrayOf(ax, ay, bx, by, if (align >= 0) 1.0 else -1.0)
            }
        }
        return best
    }

    private fun autoSnapToRoad(rawX: Double, rawY: Double, rawSpeed: Double): Matched? {
        val snapper = roadSnapper ?: return null
        val moved = hypot(rawX - lastSnapX, rawY - lastSnapY)
        val due = !autoHasEdge || moved > AUTO_SNAP_REQUERY_M ||
            elapsed - lastSnapElapsed > AUTO_SNAP_REQUERY_S
        if (due) {
            lastSnapX = rawX; lastSnapY = rawY; lastSnapElapsed = elapsed
            lastSnapHeading = vehicleHeadingDeg
            val (lat, lon) = enuToLatLon(rawX, rawY)
            val snap = try {
                snapper.snap(lat, lon, lastSnapHeading)
            } catch (_: Exception) { null }
            if (snap != null && snap.lateralM <= AUTO_SNAP_MAX_DIST_M) {
                val a = latLonToEnu(snap.aLat, snap.aLon)
                val b = latLonToEnu(snap.bLat, snap.bLon)
                if (hypot(b.first - a.first, b.second - a.second) > 1.0) {
                    autoAx = a.first; autoAy = a.second
                    autoBx = b.first; autoBy = b.second
                    autoHasEdge = true
                }
            } else {
                autoHasEdge = false
            }
        }
        val matched = if (autoHasEdge) projectOntoAutoEdge(rawX, rawY, rawSpeed) else null
        autoSnapped = matched != null
        autoSnapActive = autoSnapped
        return matched
    }

    private fun projectOntoAutoEdge(rawX: Double, rawY: Double, rawSpeed: Double): Matched? {
        val dx = autoBx - autoAx; val dy = autoBy - autoAy
        val l2 = dx * dx + dy * dy
        if (l2 < 1.0) {
            autoHasEdge = false
            return null
        }
        val tRaw = ((rawX - autoAx) * dx + (rawY - autoAy) * dy) / l2
        if (tRaw < -0.05 || tRaw > 1.05) {
            autoHasEdge = false
            return null
        }
        val t = tRaw.coerceIn(0.0, 1.0)
        val qx = autoAx + t * dx; val qy = autoAy + t * dy
        val d = hypot(rawX - qx, rawY - qy)
        if (d > AUTO_SNAP_MAX_DIST_M) {
            autoHasEdge = false
            return null
        }

        var ux = dx / sqrt(l2); var uy = dy / sqrt(l2)
        val hRad = Math.toRadians(vehicleHeadingDeg.toDouble())
        val hx = sin(hRad); val hy = cos(hRad)
        if (ux * hx + uy * hy < 0) {
            ux = -ux; uy = -uy
        }
        return Matched(qx, qy, ux * rawSpeed, uy * rawSpeed)
    }

    private fun enuToLatLon(e: Double, n: Double): Pair<Double, Double> {
        val lat = originLat + Math.toDegrees(n / 6378137.0)
        val lon = originLon + Math.toDegrees(e / (6378137.0 * cos(Math.toRadians(originLat))))
        return lat to lon
    }

    private fun latLonToEnu(lat: Double, lon: Double): Pair<Double, Double> {
        val e = Math.toRadians(lon - originLon) * 6378137.0 * cos(Math.toRadians(originLat))
        val n = Math.toRadians(lat - originLat) * 6378137.0
        return e to n
    }

    private fun projectRouteS(e: Double, n: Double): Double {
        if (route.size < 2 || routeCumDist.size != route.size) return routeS
        var best = Double.POSITIVE_INFINITY
        var bs = routeS
        for (i in 0 until route.size - 1) {
            val (ax, ay) = route[i]; val (cx, cy) = route[i + 1]
            val dx = cx - ax; val dy = cy - ay
            val segLen = routeCumDist[i + 1] - routeCumDist[i]
            if (segLen < 1e-6) continue
            val t = (((e - ax) * dx + (n - ay) * dy) / (segLen * segLen)).coerceIn(0.0, 1.0)
            val d = hypot(e - (ax + t * dx), n - (ay + t * dy))
            if (d < best) {
                best = d
                bs = routeCumDist[i] + t * segLen
            }
        }
        return bs
    }

    override fun reset() {
        state = null
        lastNanos = -1L
        elapsed = 0.0
        x = 0.0; y = 0.0
        forwardSpeed = 0.0
        lateralVel = 0.0
        ve = 0.0; vn = 0.0
        filtVe = 0.0; filtVn = 0.0; filtInit = false
        dominantHeadingDeg = 0f; dominantInit = false
        vehicleHeadingDeg = 0f
        gyroHeadingDeg = 0f; gyroHeadingInit = false
        lastDeviceYawDeg = null
        yawRateDegS = 0.0
        lpGyroMag = 0.0
        mountOffsetDeg = 0.0
        mountCalibrated = false
        biasE = 0.0; biasN = 0.0
        stillDuration = 0.0
        isStationary = false
        zuptCount = 0L
        stepCount = 0
        route.clear()
        routeCumDist = DoubleArray(0)
        routeS = 0.0
        gpsAvailable = false
        speedFailure = false
        lateralFailure = false
        status = "waiting for GNSS"
        lastAccMag = Float.NaN
        lastDebug = null
        lpAx = 0.0; lpAy = 0.0; lpAz = 0.0; lpInit = false
        lastGnssMillis = -1L
        gnssRejectStreak = 0; gnssRejects = 0
        autoHasEdge = false; autoSnapped = false; autoSnapActive = false
        lastSnapX = 0.0; lastSnapY = 0.0; lastSnapElapsed = -1.0
        trailE.clear(); trailN.clear(); trailH.clear()
        lastTrailX = Double.NaN; lastTrailY = Double.NaN
        trHasEdge = false; trReversed = false; trailSnapActive = false
        snapMode = "FREE"
        lastTrailQueryX = 0.0; lastTrailQueryY = 0.0; lastTrailQueryElapsed = -1.0
        recentRawE.clear(); recentRawN.clear()
        lastRecentX = Double.NaN; lastRecentY = Double.NaN
        lastOutX = Double.NaN; lastOutY = Double.NaN; lastMoveElapsed = 0.0; freezeLogged = false
    }

    companion object {
        /** IIR low-pass alpha on linear accel before integration (~4 Hz @100 Hz). */
        private const val ACC_LP_ALPHA = 0.25
        /** IIR low-pass alpha on gyro magnitude for the turn-energy gate. */
        private const val GYRO_LP_ALPHA = 0.1
        /** Yaw-rate EMA alpha per sample (~0.5 s time constant @100 Hz). */
        private const val YAW_RATE_ALPHA = 0.05
        /** Per-sample pull of the gyro heading toward absolute device yaw. */
        private const val GYRO_ANCHOR_W = 0.02
        /** Per-step accel magnitude clamp (m/s^2). */
        private const val ACC_MAX = 20.0
        /** Maximum forward acceleration allowed for vehicle integration (m/s^2). */
        private const val ACC_FWD_MAX = 4.5
        /** Maximum braking deceleration allowed for vehicle integration (m/s^2). */
        private const val ACC_BRAKE_MAX = 6.5
        /** Speed clamp preserving direction (m/s); recovers when samples calm down. */
        private const val V_MAX = 55.0
        /** Aerodynamic & rolling coast drag per second. */
        private const val COAST_DRAG_COEFF = 0.02
        /** Major-velocity-vector EMA time constant (s). */
        private const val VEL_FILTER_TAU_S = 0.4
        /** Below this filtered speed the velocity heading is noise: ignore (m/s). */
        private const val VEL_HEADING_MIN_SPEED = 1.5
        /** Dominant-path latch needs real motion (m/s). */
        private const val DOMINANT_MIN_SPEED = 2.5
        /** Dominant heading adaptation time constant (s): slow = stable. */
        private const val DOMINANT_TAU_S = 2.0
        /** Travel-heading pull toward the dominant vector, time constant (s). */
        private const val HEADING_PULL_TAU_S = 3.0
        /** Heading rate limiter (deg/s): a car cannot spin faster. */
        private const val HEADING_MAX_RATE_DEG_S = 100.0
        /** Above this yaw rate the vehicle is turning: suppress lateral (deg/s). */
        private const val TURN_YAW_RATE_DEG_S = 15.0
        /** Gyro-magnitude turn gate fallback when attitude is unknown (rad/s). */
        private const val TURN_GYRO_MAG_RAD_S = 0.25
        /** Lateral accel residual scale while turning (centripetal already removed). */
        private const val TURN_LAT_SUPPRESS = 0.3
        /** Lateral velocity damping, 1/s, plus yaw-rate-proportional turn term. */
        private const val LAT_BASE_DAMP = 1.5
        private const val LAT_TURN_GAIN = 2.0
        /** Sideways velocities below this while going straight are MEMS noise (m/s). */
        private const val LAT_DEADBAND_MPS = 0.25
        /** Hard bound on residual lateral velocity (m/s). */
        private const val LAT_MAX_MPS = 8.0
        /** Standstill detection gyro threshold (rad/s). */
        private const val STANDSTILL_GYRO_MAX = 0.06
        /** Standstill detection linear accel threshold (m/s^2). */
        private const val STANDSTILL_ACC_MAX = 0.35
        /** Minimum standstill duration to engage ZUPT (seconds) - 1.2s prevents cruising false-positives. */
        private const val STANDSTILL_MIN_TIME_S = 1.2
        /** GNSS gate: jump > K*accuracy + cappedSpeed*dt + MARGIN is rejected. */
        private const val GNSS_GATE_K = 3.0
        private const val GNSS_GATE_MARGIN = 15.0
        private const val GNSS_GATE_SPEED_CAP = 25.0
        /** Accept the fix anyway after this many consecutive rejects (resync). */
        private const val GNSS_REJECT_STREAK_ACCEPT = 5
        /** Blend weight k = (REF/accuracy) in [MIN, 1]: nudge vs snap. */
        private const val GNSS_BLEND_REF = 5.0
        private const val GNSS_BLEND_MIN = 0.15
        /** Fixes claiming speed above this are not adopted (m/s). */
        private const val GNSS_FIX_SPEED_MAX = 40f
        /** Bearings below this fix speed are garbage (standstill wander). */
        private const val GNSS_BEARING_MIN_SPEED = 1.5f
        /** Route projection search window ahead of current progress (m). */
        private const val ROUTE_LOOKAHEAD_M = 100.0
        /** Straight fallbacks longer than this are nonsense: drop the constraint. */
        private const val REBUILD_MAX_LEN_M = 1_000_000.0
        /** Auto road-snap: needs real motion, else hand jitter flickers roads. */
        private const val AUTO_SNAP_MIN_SPEED = 1.5f
        /** Re-query the graph after this much travel or this much time. */
        private const val AUTO_SNAP_REQUERY_M = 10.0
        private const val AUTO_SNAP_REQUERY_S = 2.0
        /** Lateral pin limit, same semantics as the planned-route gate (m). */
        private const val AUTO_SNAP_MAX_DIST_M = 30.0
        /** Trail memory: decimate, cap, and snap gates. */
        private const val TRAIL_MIN_DIST_M = 2.0
        private const val TRAIL_MAX_POINTS = 2000
        private const val TRAIL_MIN_SPEED = 1.0
        private const val TRAIL_SNAP_RADIUS_M = 20.0
        private const val TRAIL_REQUERY_M = 5.0
        private const val TRAIL_REQUERY_S = 1.5
        /** |align| above this counts as resembling the old segment (either way). */
        private const val TRAIL_MIN_ALIGN = 0.5
        private const val TRAIL_ALIGN_WEIGHT_M = 10.0
        /** Recent raw motion must span this before the resemblance gate applies (m). */
        private const val TRAIL_RESEMBLE_MIN_LEN_M = 6.0
        private const val RECENT_MIN_DIST_M = 1.0
        private const val RECENT_MAX_POINTS = 60
    }
}
