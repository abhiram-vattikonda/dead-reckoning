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
    // Low-passed linear accel (IIR) used for integration: hand/scooty
    // vibration rectifies into velocity if integrated raw (freeze fault).
    private var lpAx = 0.0; private var lpAy = 0.0; private var lpInit = false
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
    private var autoAx = 0.0; private var autoAy = 0.0
    private var autoBx = 0.0; private var autoBy = 0.0
    private var autoHasEdge = false
    private var autoSnapped = false
    private var lastSnapX = 0.0; private var lastSnapY = 0.0
    private var lastSnapElapsed = -1.0
    private var lastSnapHeading = 0f
    /** True while laterally pinned to an auto-discovered road segment. */
    var autoSnapActive = false
        private set
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
        val len = hypot(e, n).coerceAtLeast(1.0)
        // Guard: never constrain onto nonsense geometry. A destination with no
        // usable road data from a relative (0,0) origin (or any absurdly far
        // target) would otherwise drag the whole track onto an arbitrary line.
        // Clearing the route degrades to free coast / auto road-snap instead.
        if (len > REBUILD_MAX_LEN_M) { route.clear(); routeS = 0.0; return }
        route.clear(); route.add(0.0 to 0.0)
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
        // FIX 1a (freeze fault): low-pass linear accel before integration so
        // high-frequency hand/scooty shake does not rectify into velocity.
        // First-order IIR (~4 Hz corner at 100 Hz sample rate).
        if (!lpInit) { lpAx = ax; lpAy = ay; lpInit = true }
        lpAx += ACC_LP_ALPHA * (ax - lpAx)
        lpAy += ACC_LP_ALPHA * (ay - lpAy)
        // FIX 1b: clamp per-step accel magnitude; single shock spikes (pothole,
        // hand jerk) must not inject huge delta-v in one step.
        var fax = lpAx
        var fay = lpAy
        val aMag = hypot(fax, fay)
        if (aMag > ACC_MAX) { fax *= ACC_MAX / aMag; fay *= ACC_MAX / aMag }
        val qx = sample.quatX; val qy = sample.quatY; val qz = sample.quatZ; val qw = sample.quatW
        if (qx != null && qy != null && qz != null && qw != null) {
            val r = CoordinateTransformer.rotationMatrixFromQuaternion(qx, qy, qz, qw)
            val a = floatArrayOf(fax.toFloat(), fay.toFloat(), 0f); val w = FloatArray(3)
            CoordinateTransformer.transform(a, r, w); ve += w[0] * dt; vn += w[1] * dt
            yaw = atan2(ve, vn)
        }
        x += ve * dt; y += vn * dt
        val speed = hypot(ve, vn)
        // FIX 1c (freeze fault): clamp speed to V_MAX preserving direction
        // instead of zeroing it. Zeroing re-trips every step under sustained
        // shaking and deadlocks the trail; clamping degrades gracefully and
        // recovers the moment samples calm down (flag clears on plausible speed).
        speedFailure = !speed.isFinite() || speed > V_MAX
        if (!speed.isFinite()) { ve = 0.0; vn = 0.0; status = "speed failure: bounded" }
        else if (speed > V_MAX) { ve *= V_MAX / speed; vn *= V_MAX / speed; status = "speed clamped: vibration" }
        constrainToRoute()
        // Destination-less map matching: same lateral pin as a planned route,
        // but the segment is discovered from the direction of travel. Only
        // when no planned route exists (a real route always wins).
        if (route.size < 2 && destinationLat == null) autoSnapToRoad()
        val ll = GeoUtils.moveEnu(originLat, originLon, y, x)
        val next = old.copy(timestampNanos = sample.timestampNanos, latitude = ll.first, longitude = ll.second,
            velEast = ve.toFloat(), velNorth = vn.toFloat(), headingDeg = GeoUtils.normalizeHeading(Math.toDegrees(atan2(ve, vn)).toFloat()),
            confidenceMeters = (2.0 + elapsed * if (gpsAvailable) 0.05 else 1.0).toFloat())
        state = next; if (!gpsAvailable && !speedFailure) status = if (autoSnapActive) "GNSS unavailable: inertial + road snap" else "GNSS unavailable: inertial + route"
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
        val e = (g.longitude - originLon) * PI / 180 * 6378137.0 * cos(Math.toRadians(originLat)); val n = (g.latitude - originLat) * PI / 180 * 6378137.0
        // FIX 2 (teleport fault): plausibility-gate the fix before trusting it.
        // A fix implying impossible motion from the current state (multipath
        // glitch, stale reacquire) is rejected instead of hard-snapped.
        // Gate radius scales with outage duration so genuine reacquires pass.
        val jump = hypot(e - x, n - y)
        val nowMs = g.timestampMillis
        val dtGps = if (lastGnssMillis < 0) 0.0
            else ((nowMs - lastGnssMillis) / 1000.0).coerceIn(0.0, 3600.0)
        val acc = (g.accuracy ?: 5f).toDouble().coerceAtLeast(1.0)
        val gate = GNSS_GATE_K * acc + min(hypot(ve, vn), GNSS_GATE_SPEED_CAP) * dtGps + GNSS_GATE_MARGIN
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
        // Blend toward the fix (weight by reported accuracy) instead of a hard
        // snap: accurate fixes correct ~fully, sloppy ones only nudge.
        val k = (GNSS_BLEND_REF / acc).coerceIn(GNSS_BLEND_MIN, 1.0)
        x += k * (e - x); y += k * (n - y)
        // FIX 3b: resync route progress to the corrected position so the
        // windowed projection (FIX 3a) never gets stranded behind a teleport.
        routeS = projectRouteS(x, y)
        g.speed?.let {
            if (it.isFinite() && it in 0f..GNSS_FIX_SPEED_MAX) {
                if (it > GNSS_BEARING_MIN_SPEED) {
                    val h = Math.toRadians((g.bearing ?: Math.toDegrees(atan2(ve, vn)).toFloat()).toDouble())
                    ve = it * sin(h); vn = it * cos(h)
                } else if (!ve.isFinite() || !vn.isFinite()) { ve = 0.0; vn = 0.0 }
            }
        }
        state = state?.copy(latitude = originLat + Math.toDegrees(y / 6378137.0), longitude = originLon + Math.toDegrees(e / (6378137.0 * cos(Math.toRadians(originLat)))), velEast = ve.toFloat(), velNorth = vn.toFloat(), confidenceMeters = (g.accuracy ?: 5f))
        // Keep lat/lon exactly on the fix when fully trusted (k == 1).
        if (k >= 1.0) state = state?.copy(latitude = g.latitude, longitude = g.longitude)
        status = "GNSS fused: route constrained"
    }

    private fun constrainToRoute() {
        if (route.size < 2) return
        var best = Double.POSITIVE_INFINITY; var bx = x; var by = y; var bs = routeS; var tx = 1.0; var ty = 0.0
        // FIX 3a (snap trap): search only near current route progress. The old
        // full-route forward-only search could latch onto a far-ahead segment
        // where a looping road passes nearby, then monotonic routeS could never
        // come back: position pinned to one vertex while riding. Window keeps
        // legitimate per-step progress (cm-scale) free and blocks km jumps.
        for (i in 0 until route.size - 1) { val (ax, ay) = route[i]; val (cx, cy) = route[i + 1]; val dx = cx - ax; val dy = cy - ay; val l2 = dx * dx + dy * dy; if (l2 == 0.0) continue; val t = (((x - ax) * dx + (y - ay) * dy) / l2).coerceIn(0.0, 1.0); val qx = ax + t * dx; val qy = ay + t * dy; val d = hypot(x - qx, y - qy); val s = i * sqrt(l2) + t * sqrt(l2); if (s + 3 >= routeS && s <= routeS + ROUTE_LOOKAHEAD_M && d < best) { best = d; bx = qx; by = qy; bs = s; tx = dx / sqrt(l2); ty = dy / sqrt(l2) } }
        lateralFailure = best > 30.0
        if (!lateralFailure) { x = bx; y = by; routeS = bs; val sp = hypot(ve, vn); ve = tx * sp; vn = ty * sp }
    }

    /**
     * Destination-less map matching. Estimates the road under the vehicle
     * from position + travel direction, pins laterally onto it, and advances
     * along it — the planned-route constraint, discovered. Throttled graph
     * queries (~0.5 Hz / 10 m); per-step projection onto the stored edge.
     * Anything unresolved (no graph coverage, too far from any road,
     * near-standstill) degrades to free coast, never a pin.
     */
    private fun autoSnapToRoad() {
        val snapper = roadSnapper
        val sp = hypot(ve, vn)
        if (snapper == null || sp < AUTO_SNAP_MIN_SPEED) {
            // Standstill: hold the last pin (if any) without re-querying, so
            // hand jitter at traffic lights cannot flicker between roads.
            if (autoHasEdge) projectOntoAutoEdge()
            else autoSnapped = false
            autoSnapActive = autoSnapped
            return
        }
        val moved = hypot(x - lastSnapX, y - lastSnapY)
        val due = !autoHasEdge || moved > AUTO_SNAP_REQUERY_M ||
            elapsed - lastSnapElapsed > AUTO_SNAP_REQUERY_S
        if (due) {
            lastSnapX = x; lastSnapY = y; lastSnapElapsed = elapsed
            lastSnapHeading = GeoUtils.normalizeHeading(Math.toDegrees(atan2(ve, vn)).toFloat())
            val (lat, lon) = enuToLatLon(x, y)
            val snap = try {
                snapper.snap(lat, lon, lastSnapHeading)
            } catch (_: Exception) { null }
            if (snap != null && snap.lateralM <= AUTO_SNAP_MAX_DIST_M) {
                val a = latLonToEnu(snap.aLat, snap.aLon)
                val b = latLonToEnu(snap.bLat, snap.bLon)
                // Reject degenerate edges; keep the previous pin otherwise.
                if (hypot(b.first - a.first, b.second - a.second) > 1.0) {
                    autoAx = a.first; autoAy = a.second
                    autoBx = b.first; autoBy = b.second
                    autoHasEdge = true
                }
            } else {
                autoHasEdge = false
            }
        }
        if (autoHasEdge) projectOntoAutoEdge() else autoSnapped = false
        autoSnapActive = autoSnapped
    }

    /** Pin onto the stored auto edge; velocity follows the road direction. */
    private fun projectOntoAutoEdge() {
        val dx = autoBx - autoAx; val dy = autoBy - autoAy
        val l2 = dx * dx + dy * dy
        if (l2 < 1.0) { autoSnapped = false; return }
        val t = (((x - autoAx) * dx + (y - autoAy) * dy) / l2).coerceIn(0.0, 1.0)
        val qx = autoAx + t * dx; val qy = autoAy + t * dy
        val d = hypot(x - qx, y - qy)
        if (d > AUTO_SNAP_MAX_DIST_M) { autoSnapped = false; return }
        x = qx; y = qy
        // Follow the road either way: keep the travel direction, not the edge
        // orientation (roads are undirected for matching purposes).
        var ux = dx / sqrt(l2); var uy = dy / sqrt(l2)
        if (ux * ve + uy * vn < 0) { ux = -ux; uy = -uy }
        val sp = hypot(ve, vn)
        ve = ux * sp; vn = uy * sp
        autoSnapped = true
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

    /** Global nearest arc-length on the route (no progress gate): resync use. */
    private fun projectRouteS(e: Double, n: Double): Double {        var best = Double.POSITIVE_INFINITY
        var bs = routeS
        for (i in 0 until route.size - 1) {
            val (ax, ay) = route[i]; val (cx, cy) = route[i + 1]
            val dx = cx - ax; val dy = cy - ay; val l2 = dx * dx + dy * dy
            if (l2 == 0.0) continue
            val t = (((e - ax) * dx + (n - ay) * dy) / l2).coerceIn(0.0, 1.0)
            val d = hypot(e - (ax + t * dx), n - (ay + t * dy))
            if (d < best) { best = d; bs = i * sqrt(l2) + t * sqrt(l2) }
        }
        return bs
    }
    override fun reset() { state = null; lastNanos = -1L; elapsed = 0.0; x = 0.0; y = 0.0; ve = 0.0; vn = 0.0; route.clear(); routeS = 0.0; gpsAvailable = false; speedFailure = false; lateralFailure = false; status = "waiting for GNSS"; lastAccMag = Float.NaN; lastDebug = null; lpAx = 0.0; lpAy = 0.0; lpInit = false; lastGnssMillis = -1L; gnssRejectStreak = 0; gnssRejects = 0; autoHasEdge = false; autoSnapped = false; autoSnapActive = false; lastSnapElapsed = -1.0 }

    companion object {
        /** IIR low-pass alpha on linear accel before integration (~4 Hz @100 Hz). */
        private const val ACC_LP_ALPHA = 0.25
        /** Per-step accel magnitude clamp (m/s^2): shock spikes never inject huge delta-v. */
        private const val ACC_MAX = 20.0
        /** Speed clamp preserving direction (m/s); recovers when samples calm down. */
        private const val V_MAX = 55.0
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
        private const val AUTO_SNAP_MIN_SPEED = 2.0f
        /** Re-query the graph after this much travel or this much time. */
        private const val AUTO_SNAP_REQUERY_M = 10.0
        private const val AUTO_SNAP_REQUERY_S = 2.0
        /** Lateral pin limit, same semantics as the planned-route gate (m). */
        private const val AUTO_SNAP_MAX_DIST_M = 30.0
    }
}
