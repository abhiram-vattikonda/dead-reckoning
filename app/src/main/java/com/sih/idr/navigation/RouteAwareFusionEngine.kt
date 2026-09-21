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
 * 2. Direct IMU orientation heading tracking: decouples vehicle yaw from noisy integrated
 *    velocity vectors, using the device gyroscope/rotation vector with automatic mount calibration.
 * 3. Zero Velocity Updates (ZUPT): monitors standstill intervals to reset velocity to zero and
 *    cancel accelerometer bias, arresting the primary cause of explosive drift at traffic stops.
 * 4. Non-holonomic vehicle motion constraints: projects acceleration along travel heading and
 *    suppresses orthogonal lateral velocity drift.
 * 5. Exact cumulative arc-length route constraint: computes true metric progress along routes
 *    with variable segment lengths, eliminating lookahead window lock-ups.
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
    private var ve = 0.0
    private var vn = 0.0

    // Heading state in degrees clockwise from North [0, 360).
    var vehicleHeadingDeg = 0f
        private set

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
        val hRad = Math.toRadians(initialState.headingDeg.toDouble())
        ve = forwardSpeed * sin(hRad)
        vn = forwardSpeed * cos(hRad)
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
                az = (sample.accelZ - 9.80665).toDouble()
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

        // 3. Orientation & Vehicle Heading from Gyro/Rotation-Vector
        if (rotM != null) {
            val ypr = CoordinateTransformer.yawPitchRollDeg(rotM)
            val deviceYaw = ypr[0].toDouble()
            vehicleHeadingDeg = GeoUtils.normalizeHeading((deviceYaw + mountOffsetDeg).toFloat())
        }

        val yawRad = Math.toRadians(vehicleHeadingDeg.toDouble())
        val cosH = cos(yawRad)
        val sinH = sin(yawRad)

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
            aEast = fax * sinH + fay * cosH
            aNorth = -fax * cosH + fay * sinH
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

        if (isStationary) {
            // ZUPT active: zero velocity and calibrate zero-g bias
            forwardSpeed = 0.0
            ve = 0.0
            vn = 0.0
            biasE += 0.05 * (aEast - biasE)
            biasN += 0.05 * (aNorth - biasN)
            zuptCount++
            status = "Stationary (ZUPT active)"
        } else {
            // 6. Longitudinal Forward Acceleration & Velocity Propagation
            if (isStep) {
                // Pedestrian step impulse (~1.25 m/s cadence)
                if (forwardSpeed < 1.25) {
                    forwardSpeed = 1.25
                }
            }

            val aEffEast = aEast - biasE
            val aEffNorth = aNorth - biasN

            // Longitudinal acceleration along travel heading
            val aFwd = (aEffEast * sinH + aEffNorth * cosH).coerceIn(-ACC_BRAKE_MAX, ACC_FWD_MAX)

            forwardSpeed += aFwd * dt

            // Natural aerodynamic & rolling damping when coasting
            if (abs(aFwd) < 0.15 && !isStep) {
                forwardSpeed *= (1.0 - COAST_DRAG_COEFF * dt)
            }

            speedFailure = !forwardSpeed.isFinite() || forwardSpeed > V_MAX
            if (!forwardSpeed.isFinite()) {
                forwardSpeed = 0.0
                status = "speed failure: bounded"
            } else if (forwardSpeed > V_MAX) {
                forwardSpeed = V_MAX
                status = "speed clamped: bound"
            }

            forwardSpeed = forwardSpeed.coerceAtLeast(0.0)

            ve = forwardSpeed * sinH
            vn = forwardSpeed * cosH

            val fresh = isGnssFresh()
            gpsAvailable = fresh
            if (!fresh) {
                status = if (autoSnapActive) "GPS OFF: inertial + road snap"
                else if (route.size >= 2 && !lateralFailure) "GPS OFF: inertial + route"
                else if (isStep) "GPS OFF: pedestrian DR (walking)"
                else "GPS OFF: inertial dead reckoning"
            }
        }

        // 7. Advance RAW position
        x += ve * dt
        y += vn * dt

        // 8. Map Matching / Snapping on a Copy
        var outX = x; var outY = y; var outVe = ve; var outVn = vn
        val routeMatch = constrainToRoute(x, y, forwardSpeed)
        if (routeMatch != null) {
            outX = routeMatch.x; outY = routeMatch.y
            outVe = routeMatch.ve; outVn = routeMatch.vn
            routeS = routeMatch.routeS
        } else {
            val snap = autoSnapToRoad(x, y, forwardSpeed)
            if (snap != null) {
                outX = snap.x; outY = snap.y
                outVe = snap.ve; outVn = snap.vn
            }
        }

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

        // Engine debug reporter snapshot (for CSV export and evaluation)
        lastDebug = com.sih.idr.navigation.pdr.PdrDebug(
            timestampNanos = sample.timestampNanos,
            accMagnitude = lastAccMag,
            stepDetected = isStep,
            strideLength = if (isStep) 0.75f else Float.NaN,
            gyroHeadingRad = Math.toRadians(vehicleHeadingDeg.toDouble()).toFloat(),
            magHeadingRad = Float.NaN,
            fusedHeadingRad = Math.toRadians(outHeadingDeg.toDouble()).toFloat(),
            xEast = outX.toFloat(),
            yNorth = outY.toFloat(),
            latitude = ll.first,
            longitude = ll.second
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
                    val hRad = Math.toRadians(bearingDeg)
                    ve = forwardSpeed * sin(hRad)
                    vn = forwardSpeed * cos(hRad)
                } else if (spd < 0.5f) {
                    forwardSpeed = 0.0
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

    internal fun constrainToRoute(rawX: Double, rawY: Double, rawSpeed: Double): Matched? {
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
        ve = 0.0; vn = 0.0
        vehicleHeadingDeg = 0f
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
        lastOutX = Double.NaN; lastOutY = Double.NaN; lastMoveElapsed = 0.0; freezeLogged = false
    }

    companion object {
        /** IIR low-pass alpha on linear accel before integration (~4 Hz @100 Hz). */
        private const val ACC_LP_ALPHA = 0.25
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
    }
}
