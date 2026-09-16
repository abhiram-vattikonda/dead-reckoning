package com.sih.idr.navigation

import com.sih.idr.data.GnssMeasurement
import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState
import com.sih.idr.utils.GeoUtils
import kotlin.math.*

/** Vehicle engine: inertial propagation + GNSS correction + ordered route constraint. */
class RouteAwareFusionEngine : DeadReckoningEngine, EngineDebugReporter {
    private var state: NavigationState? = null
    private var lastNanos = -1L
    private var elapsed = 0.0
    private var originLat = 0.0; private var originLon = 0.0
    private var x = 0.0; private var y = 0.0
    private var ve = 0.0; private var vn = 0.0
    private var yaw = 0.0
    private var lastAccMag = Float.NaN
    override var lastDebug: com.sih.idr.navigation.pdr.PdrDebug? = null
        private set
    private var route: MutableList<Pair<Double, Double>> = mutableListOf()
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
        destinationLat = latitude; destinationLon = longitude
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
        route.clear(); routeS = 0.0
        for ((lat, lon) in points) {
            val e = Math.toRadians(lon - originLon) * 6378137.0 * cos(Math.toRadians(originLat))
            val n = Math.toRadians(lat - originLat) * 6378137.0
            route.add(e to n)
        }
        if (route.size < 2) rebuildRoute()
    }
    private fun rebuildRoute() {
        val latitude = destinationLat ?: return
        val longitude = destinationLon ?: return
        val e = (longitude - originLon) * PI / 180.0 * 6378137.0 * cos(Math.toRadians(originLat))
        val n = (latitude - originLat) * PI / 180.0 * 6378137.0
        route.clear(); route.add(0.0 to 0.0)
        val len = hypot(e, n).coerceAtLeast(1.0)
        val steps = ceil(len / 10.0).toInt().coerceIn(2, 2000)
        for (i in 1..steps) { val f = i.toDouble() / steps; route.add(e * f to n * f) }
        routeS = 0.0
    }
    fun clearRoute() { route.clear(); routeS = 0.0; destinationLat = null; destinationLon = null }
    fun routeLatLon(): List<Pair<Double, Double>> = route.map { GeoUtils.moveEnu(originLat, originLon, it.second, it.first) }

    override fun initialize(initialState: NavigationState) {
        reset(); state = initialState; originLat = initialState.latitude; originLon = initialState.longitude
        ve = initialState.velEast.toDouble(); vn = initialState.velNorth.toDouble()
        yaw = Math.toRadians(initialState.headingDeg.toDouble()); status = "GNSS anchor acquired"
        pendingRouteLatLon?.let { applyRouteLatLon(it) } ?: rebuildRoute()
    }

    override fun processImu(sample: ImuSample): NavigationState {
        val old = state ?: return NavigationState(sample.timestampNanos, 0.0, 0.0)
        val dt = if (lastNanos < 0) 0.01 else ((sample.timestampNanos - lastNanos) / 1e9).coerceIn(0.001, 0.2)
        lastNanos = sample.timestampNanos; elapsed += dt
        val ax = sample.linearX?.toDouble() ?: 0.0
        val ay = sample.linearY?.toDouble() ?: 0.0
        val az = sample.linearZ?.toDouble() ?: 0.0
        lastAccMag = sqrt(ax * ax + ay * ay + az * az).toFloat()
        val qx = sample.quatX; val qy = sample.quatY; val qz = sample.quatZ; val qw = sample.quatW
        if (qx != null && qy != null && qz != null && qw != null) {
            val r = CoordinateTransformer.rotationMatrixFromQuaternion(qx, qy, qz, qw)
            val a = floatArrayOf(ax.toFloat(), ay.toFloat(), 0f); val w = FloatArray(3)
            CoordinateTransformer.transform(a, r, w); ve += w[0] * dt; vn += w[1] * dt
            yaw = atan2(ve, vn)
        }
        x += ve * dt; y += vn * dt
        val speed = hypot(ve, vn)
        speedFailure = !speed.isFinite() || speed > 55.0
        if (speedFailure) { ve = 0.0; vn = 0.0; status = "speed failure: bounded" }
        constrainToRoute()
        val ll = GeoUtils.moveEnu(originLat, originLon, y, x)
        val next = old.copy(timestampNanos = sample.timestampNanos, latitude = ll.first, longitude = ll.second,
            velEast = ve.toFloat(), velNorth = vn.toFloat(), headingDeg = GeoUtils.normalizeHeading(Math.toDegrees(atan2(ve, vn)).toFloat()),
            confidenceMeters = (2.0 + elapsed * if (gpsAvailable) 0.05 else 1.0).toFloat())
        state = next; if (!gpsAvailable && !speedFailure) status = "GNSS unavailable: inertial + route"
        // Engine-agnostic debug snapshot (no step model here: step/stride/mag stay NaN/false).
        lastDebug = com.sih.idr.navigation.pdr.PdrDebug(
            timestampNanos = sample.timestampNanos,
            accMagnitude = lastAccMag,
            stepDetected = false,
            strideLength = Float.NaN,
            gyroHeadingRad = yaw.toFloat(),
            magHeadingRad = Float.NaN,
            fusedHeadingRad = yaw.toFloat(),
            xEast = x.toFloat(),
            yNorth = y.toFloat(),
            latitude = ll.first,
            longitude = ll.second
        )
        return next
    }

    override fun onGnssMeasurement(measurement: GnssMeasurement) {
        val g = measurement.sample; if ((g.accuracy ?: 0f) > 30f) return
        gpsAvailable = true; val e = (g.longitude - originLon) * PI / 180 * 6378137.0 * cos(Math.toRadians(originLat)); val n = (g.latitude - originLat) * PI / 180 * 6378137.0
        x = e; y = n; g.speed?.let { if (it.isFinite() && it >= 0f) { val h = Math.toRadians((g.bearing ?: Math.toDegrees(atan2(ve, vn)).toFloat()).toDouble()); ve = it * sin(h); vn = it * cos(h) } }
        state = state?.copy(latitude = g.latitude, longitude = g.longitude, velEast = ve.toFloat(), velNorth = vn.toFloat(), confidenceMeters = (g.accuracy ?: 5f))
        status = "GNSS fused: route constrained"
    }

    private fun constrainToRoute() {
        if (route.size < 2) return
        var best = Double.POSITIVE_INFINITY; var bx = x; var by = y; var bs = routeS; var tx = 1.0; var ty = 0.0
        for (i in 0 until route.size - 1) { val (ax, ay) = route[i]; val (cx, cy) = route[i + 1]; val dx = cx - ax; val dy = cy - ay; val l2 = dx * dx + dy * dy; val t = ((x - ax) * dx + (y - ay) * dy / l2).coerceIn(0.0, 1.0); val qx = ax + t * dx; val qy = ay + t * dy; val d = hypot(x - qx, y - qy); val s = i * sqrt(l2) + t * sqrt(l2); if (s + 3 >= routeS && d < best) { best = d; bx = qx; by = qy; bs = s; tx = dx / sqrt(l2); ty = dy / sqrt(l2) } }
        lateralFailure = best > 30.0
        if (!lateralFailure) { x = bx; y = by; routeS = bs; val sp = hypot(ve, vn); ve = tx * sp; vn = ty * sp }
    }
    override fun reset() { state = null; lastNanos = -1L; elapsed = 0.0; x = 0.0; y = 0.0; ve = 0.0; vn = 0.0; route.clear(); routeS = 0.0; gpsAvailable = false; speedFailure = false; lateralFailure = false; status = "waiting for GNSS"; lastAccMag = Float.NaN; lastDebug = null }
}
