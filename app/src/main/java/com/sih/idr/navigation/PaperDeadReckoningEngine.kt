package com.sih.idr.navigation

import com.sih.idr.data.GnssMeasurement
import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState
import com.sih.idr.navigation.pdr.PdrDebug
import com.sih.idr.utils.GeoUtils
import kotlin.math.*

/**
 * Vehicular Dead Reckoning Engine implemented from:
 * "Dead Reckoning on Smartphones to Reduce GPS Usage"
 * Nawarathne, Zhao, Camara Pereira, Luo (ICARCV 2014)
 *
 * System Pipeline:
 *  1. Dynamic horizontal acceleration calculation (Hemminki / Mizell / Android sensor fusion)
 *  2. Noise reduction via 4 Hz low-pass filter & 32-point STFT 1 Hz motion energy filtering (Section III-A)
 *  3. Standstill ZUPT detection arresting velocity drift at traffic stops and idle intervals
 *  4. Velocity & displacement integration in the phone reference frame (Section III-C)
 *  5. GPS velocity decomposition at speed >= 4 km/h (1.11 m/s): vgps_x = vgps*sin(beta-alpha), vgps_y = vgps*cos(beta-alpha)
 *  6. Instantaneous displacement angle theta = atan2(dx, dy) w.r.t phone's Y axis (Section III-D)
 *  7. Earth-frame heading phi = alpha + theta (alpha = phone azimuth)
 *  8. Incremental latitude/longitude projection with optional route/road constraint
 */
class PaperDeadReckoningEngine : DeadReckoningEngine, EngineDebugReporter {

    private var state: NavigationState? = null
    private var lastNanos = -1L
    private var elapsed = 0.0

    // Coordinates of current position
    private var currentLat = 0.0
    private var currentLon = 0.0
    private var currentAlt = 0.0

    // Velocities in PHONE frame (XP, YP) as defined in Section III-C
    var vx = 0.0
        private set
    var vy = 0.0
        private set

    val forwardSpeed: Double
        get() = hypot(vx, vy)

    // Heading angles (degrees clockwise from North [0, 360))
    var phoneAzimuthDeg: Float = 0f
        private set
    var travelHeadingDeg: Float = 0f
        private set

    // Filtering component (Section III-A)
    val signalFilter = PaperSignalFilter(
        motionThreshold = 0.18,
        lpfCutoffHz = 4.0,
        stillGyroThreshold = 0.08
    )

    // Status properties for UI
    var status: String = "waiting for GNSS"
        private set
    var gpsAvailable = false
        private set
    var isStationary: Boolean = false
        private set
    var mountCalibrated: Boolean = true // Handled directly via instantaneous angle theta
        private set
    var speedFailure: Boolean = false
        private set
    var lateralFailure: Boolean = false
        private set
    var stepCount: Int = 0
        private set

    // Route constraint & road snapping support
    var roadSnapper: RoadSnapProvider? = null
    var autoSnapActive = false
        private set

    private var route: MutableList<Pair<Double, Double>> = mutableListOf()
    private var routeCumDist: DoubleArray = DoubleArray(0)
    private var destinationLat: Double? = null
    private var destinationLon: Double? = null
    private var pendingRouteLatLon: List<Pair<Double, Double>>? = null
    private var routeS = 0.0

    // GNSS state
    private var lastGnssMillis = -1L
    private var lastFixSpeed = 0f

    // Low pass gravity fallback
    private var gravX = 0.0
    private var gravY = 0.0
    private var gravZ = 9.80665
    private var gravInit = false

    override var lastDebug: PdrDebug? = null
        private set

    override val currentState: NavigationState? get() = state

    override fun initialize(initialState: NavigationState) {
        reset()
        state = initialState
        currentLat = initialState.latitude
        currentLon = initialState.longitude
        currentAlt = initialState.altitude
        travelHeadingDeg = initialState.headingDeg
        phoneAzimuthDeg = initialState.headingDeg

        val initialSpeed = initialState.speed.toDouble()
        // Initialize phone Y velocity along travel direction
        vy = initialSpeed
        vx = 0.0

        status = "GNSS anchor acquired (v=%.1f m/s)".format(initialSpeed)
        pendingRouteLatLon?.let { applyRouteLatLon(it) } ?: rebuildRoute()
    }

    override fun processImu(sample: ImuSample): NavigationState {
        val old = state ?: return NavigationState(sample.timestampNanos, 0.0, 0.0)
        val dt = if (lastNanos < 0) 0.01 else ((sample.timestampNanos - lastNanos) / 1e9).coerceIn(0.001, 0.2)
        lastNanos = sample.timestampNanos
        elapsed += dt

        // 1. Phone Azimuth alpha (Section III-C, III-D)
        val qx = sample.quatX; val qy = sample.quatY; val qz = sample.quatZ; val qw = sample.quatW
        val rotM = if (qx != null && qy != null && qz != null && qw != null) {
            CoordinateTransformer.rotationMatrixFromQuaternion(qx, qy, qz, qw)
        } else null

        if (rotM != null) {
            val ypr = CoordinateTransformer.yawPitchRollDeg(rotM)
            phoneAzimuthDeg = ypr[0]
        }

        // 2. Dynamic horizontal accelerations with gravity removed (Section III-B)
        var dynAx: Double
        var dynAy: Double

        if (sample.linearX != null && sample.linearY != null) {
            dynAx = sample.linearX.toDouble()
            dynAy = sample.linearY.toDouble()
        } else if (rotM != null) {
            val gPhone = FloatArray(3)
            CoordinateTransformer.gravityInPhoneFrame(rotM, gPhone)
            dynAx = (sample.accelX - gPhone[0]).toDouble()
            dynAy = (sample.accelY - gPhone[1]).toDouble()
        } else if (sample.gravityX != null && sample.gravityY != null) {
            dynAx = (sample.accelX - sample.gravityX).toDouble()
            dynAy = (sample.accelY - sample.gravityY).toDouble()
        } else {
            // Low pass gravity fallback
            if (!gravInit) {
                gravX = sample.accelX.toDouble()
                gravY = sample.accelY.toDouble()
                gravZ = sample.accelZ.toDouble()
                gravInit = true
            } else {
                val gAlpha = 0.05
                gravX += gAlpha * (sample.accelX - gravX)
                gravY += gAlpha * (sample.accelY - gravY)
                gravZ += gAlpha * (sample.accelZ - gravZ)
            }
            dynAx = sample.accelX - gravX
            dynAy = sample.accelY - gravY
        }

        // Clamp shock spikes (potholes, impacts)
        val dynMag = hypot(dynAx, dynAy)
        if (dynMag > ACC_MAX && dynMag > 0.0) {
            val scale = ACC_MAX / dynMag
            dynAx *= scale
            dynAy *= scale
        }

        // 3. Signal Filtering & 1 Hz STFT Motion Threshold (Section III-A)
        val gyroRate = sqrt(sample.gyroX * sample.gyroX + sample.gyroY * sample.gyroY + sample.gyroZ * sample.gyroZ).toDouble()
        val filterResult = signalFilter.process(
            rawAx = dynAx,
            rawAy = dynAy,
            gyroRate = gyroRate,
            dt = dt,
            currentTimeS = elapsed
        )

        isStationary = filterResult.isStandstill
        val effectiveAx = filterResult.filteredAx
        val effectiveAy = filterResult.filteredAy

        var deltaDx: Double
        var deltaDy: Double

        if (isStationary) {
            // Zero-velocity update: vehicle is stationary
            vx = 0.0
            vy = 0.0
            deltaDx = 0.0
            deltaDy = 0.0
            status = "Stationary (ZUPT active)"
        } else {
            // 4. Integration in Phone Frame (Section III-C)
            // Displacement: sx = vx*dt + 0.5*ax*dt^2
            deltaDx = vx * dt + 0.5 * effectiveAx * dt * dt
            deltaDy = vy * dt + 0.5 * effectiveAy * dt * dt

            // Velocity update: vx += ax*dt
            vx += effectiveAx * dt
            vy += effectiveAy * dt

            // Light drag damping when cruising with low acceleration
            if (!filterResult.isMotionActive) {
                vx *= (1.0 - COAST_DRAG_COEFF * dt)
                vy *= (1.0 - COAST_DRAG_COEFF * dt)
            }

            // Speed clamping
            val currentSpeed = hypot(vx, vy)
            if (currentSpeed > V_MAX) {
                val s = V_MAX / currentSpeed
                vx *= s
                vy *= s
                speedFailure = true
            } else {
                speedFailure = false
            }

            status = if (isGnssFresh()) "GNSS active · DR tracking"
            else if (filterResult.isMotionActive) "DR tracking (1Hz STFT motion active)"
            else "DR cruising (bias filtered)"
        }

        val stepDist = hypot(deltaDx, deltaDy)

        // 5. Heading Estimation: theta = atan2(deltaDx, deltaDy), phi = alpha + theta (Section III-D)
        if (stepDist > 0.001) {
            // Instantaneous angle w.r.t phone's Y axis
            val thetaRad = atan2(deltaDx, deltaDy)
            val thetaDeg = Math.toDegrees(thetaRad)
            travelHeadingDeg = GeoUtils.normalizeHeading((phoneAzimuthDeg.toDouble() + thetaDeg).toFloat())
        }

        // 6. Convert displacement to Earth frame (East, North)
        val phiRad = Math.toRadians(travelHeadingDeg.toDouble())
        val deltaEast = stepDist * sin(phiRad)
        val deltaNorth = stepDist * cos(phiRad)

        // 7. Advance Position
        val rawNextLatLon = GeoUtils.moveEnu(currentLat, currentLon, deltaNorth, deltaEast)
        var nextLat = rawNextLatLon.first
        var nextLon = rawNextLatLon.second

        // 8. Optional Route Constraint (keeps vehicle on planned route when available)
        if (route.size >= 2) {
            val snapped = constrainToRoute(nextLat, nextLon, hypot(vx, vy))
            if (snapped != null) {
                nextLat = snapped.first
                nextLon = snapped.second
            }
        }

        currentLat = nextLat
        currentLon = nextLon

        val velEast = (hypot(vx, vy) * sin(phiRad)).toFloat()
        val velNorth = (hypot(vx, vy) * cos(phiRad)).toFloat()

        val nextState = old.copy(
            timestampNanos = sample.timestampNanos,
            latitude = currentLat,
            longitude = currentLon,
            altitude = currentAlt,
            velEast = velEast,
            velNorth = velNorth,
            headingDeg = travelHeadingDeg,
            confidenceMeters = (1.5 + elapsed * if (isGnssFresh()) 0.05 else 0.4).toFloat()
        )
        state = nextState

        lastDebug = PdrDebug(
            timestampNanos = sample.timestampNanos,
            accMagnitude = dynMag.toFloat(),
            stepDetected = false,
            strideLength = Float.NaN,
            gyroHeadingRad = Math.toRadians(phoneAzimuthDeg.toDouble()).toFloat(),
            magHeadingRad = Float.NaN,
            fusedHeadingRad = Math.toRadians(travelHeadingDeg.toDouble()).toFloat(),
            xEast = deltaEast.toFloat(),
            yNorth = deltaNorth.toFloat(),
            latitude = currentLat,
            longitude = currentLon
        )

        return nextState
    }

    /**
     * Handles external GNSS fix according to Section III-C:
     * Integration starts/re-anchors from a GPS location with minimum velocity of 4 km/h (1.11 m/s).
     * GPS velocity is decomposed into phone's X and Y axes:
     *   vgps_x = vgps * sin(beta - alpha)
     *   vgps_y = vgps * cos(beta - alpha)
     */
    override fun onGnssMeasurement(measurement: GnssMeasurement) {
        val g = measurement.sample
        val nowMs = g.timestampMillis
        lastGnssMillis = nowMs
        gpsAvailable = true

        val accuracy = (g.accuracy ?: 10f).toDouble()
        if (accuracy > 30.0) return // Skip poor GPS fixes

        val spd = g.speed ?: 0f
        lastFixSpeed = spd

        // Paper Section III-C: 4 km/h threshold (1.11 m/s) for valid GPS bearing
        if (spd >= GPS_MIN_SPEED_MPS && g.bearing != null) {
            val beta = g.bearing.toDouble()
            val alpha = phoneAzimuthDeg.toDouble()
            val angleDiffRad = Math.toRadians(beta - alpha)

            // Phone-frame velocity decomposition from Section III-C
            vx = spd * sin(angleDiffRad)
            vy = spd * cos(angleDiffRad)
            travelHeadingDeg = GeoUtils.normalizeHeading(beta.toFloat())
        } else if (spd < 0.3f) {
            // Standstill GPS fix
            vx = 0.0
            vy = 0.0
        }

        // Anchor position to GPS proportional to accuracy
        val blendK = (5.0 / accuracy).coerceIn(0.2, 1.0)
        currentLat += blendK * (g.latitude - currentLat)
        currentLon += blendK * (g.longitude - currentLon)
        g.altitude?.let { currentAlt = it }

        state = state?.copy(
            latitude = currentLat,
            longitude = currentLon,
            altitude = currentAlt,
            confidenceMeters = accuracy.toFloat()
        )
    }

    fun setDestination(latitude: Double, longitude: Double) {
        destinationLat = latitude
        destinationLon = longitude
        if (state != null) rebuildRoute()
    }

    fun setRouteLatLon(points: List<Pair<Double, Double>>) {
        pendingRouteLatLon = points.toList()
        if (state != null) applyRouteLatLon(points)
    }

    private fun applyRouteLatLon(points: List<Pair<Double, Double>>) {
        route.clear()
        routeS = 0.0
        route.addAll(points)
        updateRouteDistances()
        if (route.size < 2) rebuildRoute()
    }

    private fun rebuildRoute() {
        val lat = destinationLat ?: return
        val lon = destinationLon ?: return
        route.clear()
        route.add(currentLat to currentLon)
        val distM = GeoUtils.haversineMeters(currentLat, currentLon, lat, lon)
        if (distM > 100_000.0) {
            updateRouteDistances()
            return
        }
        val steps = ceil(distM / 15.0).toInt().coerceIn(2, 1000)
        for (i in 1..steps) {
            val f = i.toDouble() / steps
            route.add((currentLat + f * (lat - currentLat)) to (currentLon + f * (lon - currentLon)))
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
            val d = GeoUtils.haversineMeters(route[i].first, route[i].second, route[i + 1].first, route[i + 1].second)
            routeCumDist[i + 1] = routeCumDist[i] + d
        }
    }

    fun clearRoute() {
        route.clear()
        routeCumDist = DoubleArray(0)
        routeS = 0.0
        destinationLat = null
        destinationLon = null
        pendingRouteLatLon = null
    }

    private fun constrainToRoute(lat: Double, lon: Double, speed: Double): Pair<Double, Double>? {
        if (route.size < 2 || routeCumDist.size != route.size) return null
        var bestDist = Double.POSITIVE_INFINITY
        var bestLat = lat
        var bestLon = lon
        var found = false

        for (i in 0 until route.size - 1) {
            val p1 = route[i]
            val p2 = route[i + 1]
            val segLen = routeCumDist[i + 1] - routeCumDist[i]
            if (segLen < 1.0) continue

            // Distance to segment
            val d1 = GeoUtils.haversineMeters(lat, lon, p1.first, p1.second)
            val d2 = GeoUtils.haversineMeters(lat, lon, p2.first, p2.second)
            val cross = min(d1, d2)
            if (cross < bestDist && cross < ROUTE_SNAP_MAX_DIST_M) {
                bestDist = cross
                // Linear interpolation projection
                val t = (((lat - p1.first) * (p2.first - p1.first) + (lon - p1.second) * (p2.second - p1.second)) /
                    ((p2.first - p1.first).pow(2) + (lon - p1.second).pow(2).coerceAtLeast(1e-10))).coerceIn(0.0, 1.0)
                bestLat = p1.first + t * (p2.first - p1.first)
                bestLon = p1.second + t * (p2.second - p1.second)
                found = true
            }
        }

        if (found) {
            lateralFailure = false
            // Soft blend: 75% snapped, 25% dead reckoning to avoid rigid lock-ups
            val blendedLat = currentLat + (bestLat - currentLat) * 0.75
            val blendedLon = currentLon + (bestLon - currentLon) * 0.75
            return blendedLat to blendedLon
        } else {
            lateralFailure = true
            return null
        }
    }

    private fun isGnssFresh(): Boolean =
        lastGnssMillis > 0 && (System.currentTimeMillis() - lastGnssMillis) < 6000L

    override fun reset() {
        state = null
        lastNanos = -1L
        elapsed = 0.0
        currentLat = 0.0
        currentLon = 0.0
        currentAlt = 0.0
        vx = 0.0
        vy = 0.0
        phoneAzimuthDeg = 0f
        travelHeadingDeg = 0f
        signalFilter.reset()
        status = "waiting for GNSS"
        gpsAvailable = false
        isStationary = false
        speedFailure = false
        lateralFailure = false
        stepCount = 0
        lastGnssMillis = -1L
        lastFixSpeed = 0f
        gravInit = false
        route.clear()
        routeCumDist = DoubleArray(0)
        routeS = 0.0
        lastDebug = null
    }

    companion object {
        /** Minimum GPS speed to initialize bearing & velocity in m/s (4 km/h = 1.111 m/s from Section III-C). */
        const val GPS_MIN_SPEED_MPS = 1.111f
        /** Maximum acceleration magnitude clamped per step (m/s^2). */
        private const val ACC_MAX = 15.0
        /** Maximum plausible vehicle speed (50 m/s = 180 km/h). */
        private const val V_MAX = 50.0
        /** Coasting drag coefficient per second when acceleration is zeroed. */
        private const val COAST_DRAG_COEFF = 0.03
        /** Route snapping threshold in meters. */
        private const val ROUTE_SNAP_MAX_DIST_M = 40.0
    }
}
