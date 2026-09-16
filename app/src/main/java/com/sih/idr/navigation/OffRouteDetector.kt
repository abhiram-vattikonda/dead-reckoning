package com.sih.idr.navigation

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot

/**
 * Detects when the vehicle leaves the planned route and decides when to recalculate.
 * Pure Kotlin — no Android dependencies, fully unit-testable.
 *
 * Design for GPS-denied operation:
 * - Position input is normally the DR (inertial) estimate, so detection works with
 *   phone location OFF and without any network. A fresh GPS fix is preferred when
 *   available, but never required.
 * - Turn confirmation uses IMU-derived heading (engine yaw), not GPS bearing:
 *   a sustained heading change away from the route bearing distinguishes a real
 *   turn from lateral drift/noise before an expensive graph re-route is triggered.
 *
 * Decision logic per check (checks run ~every [OffRouteConfig.checkIntervalMs]):
 * - OFF_ROUTE when cross-track distance to the planned polyline > threshold,
 *   sustained over [OffRouteConfig.sustainChecks] consecutive checks, OR
 * - TURN: heading changed by > [OffRouteConfig.turnAngleDeg] within
 *   [OffRouteConfig.turnWindowMs] AND distance > 60% of threshold (fast path).
 * - Re-entering within half the threshold clears the counter; a cooldown after
 *   each recalculation prevents thrash. Checks below [OffRouteConfig.minSpeedMps]
 *   are ignored (a stationary vehicle is not "off route").
 */
data class OffRouteConfig(
    /** Lateral deviation (m) that counts as off-route. Matches engine's 30 m fault band. */
    val deviationThresholdM: Double = 35.0,
    /** Consecutive off-route checks required before recalculating. */
    val sustainChecks: Int = 4,
    /** Minimum time between checks (ms). */
    val checkIntervalMs: Long = 2000L,
    /** Heading change (deg) that counts as a deliberate turn. */
    val turnAngleDeg: Double = 30.0,
    /** Window (ms) over which the turn angle is measured. */
    val turnWindowMs: Long = 4000L,
    /** Below this speed (m/s) the vehicle is considered stationary: hold. */
    val minSpeedMps: Double = 1.0,
    /** Quiet period after a recalculation (ms). */
    val cooldownMs: Long = 20_000L
)

sealed interface OffRouteDecision {
    data object Hold : OffRouteDecision
    data class Recalculate(val deviationM: Double, val turnDeg: Double) : OffRouteDecision
}

class OffRouteDetector(val config: OffRouteConfig = OffRouteConfig()) {

    private var lastCheckMs: Long = -1L
    private var offRouteStreak = 0
    private var cooldownUntilMs: Long = -1L
    private var refHeadingDeg: Double? = null
    private var refHeadingMs: Long = -1L
    var recalculations: Int = 0
        private set
    var lastDeviationM: Double = 0.0
        private set

    fun reset() {
        lastCheckMs = -1L
        offRouteStreak = 0
        cooldownUntilMs = -1L
        refHeadingDeg = null
        refHeadingMs = -1L
        recalculations = 0
        lastDeviationM = 0.0
    }

    /**
     * @param route planned route as (lat, lon) — must contain ≥ 2 points.
     * @return [OffRouteDecision.Recalculate] when a new route should be computed.
     */
    fun update(
        nowMs: Long,
        lat: Double,
        lon: Double,
        headingDeg: Double,
        speedMps: Double,
        route: List<Pair<Double, Double>>
    ): OffRouteDecision {
        if (route.size < 2) return OffRouteDecision.Hold
        if (nowMs - lastCheckMs < config.checkIntervalMs && lastCheckMs >= 0) {
            return OffRouteDecision.Hold
        }
        lastCheckMs = nowMs
        if (nowMs < cooldownUntilMs) return OffRouteDecision.Hold
        if (speedMps < config.minSpeedMps || !speedMps.isFinite()) {
            offRouteStreak = 0
            return OffRouteDecision.Hold
        }

        val (distM, _) = nearestDistanceAndBearing(lat, lon, route)
        lastDeviationM = distM

        // Turn measurement: compare current IMU heading against the reference
        // latched turnWindowMs ago (sliding reference, refreshed continuously).
        val refH = refHeadingDeg
        if (refH == null || nowMs - refHeadingMs >= config.turnWindowMs) {
            refHeadingDeg = headingDeg
            refHeadingMs = nowMs
        }
        val turnDeg = abs(angleDiffDeg(headingDeg, refHeadingDeg ?: headingDeg))

        return if (distM > config.deviationThresholdM) {
            offRouteStreak++
            val turnConfirmed = turnDeg >= config.turnAngleDeg &&
                distM > config.deviationThresholdM * 0.6
            if (offRouteStreak >= config.sustainChecks || turnConfirmed) {
                offRouteStreak = 0
                cooldownUntilMs = nowMs + config.cooldownMs
                recalculations++
                OffRouteDecision.Recalculate(distM, turnDeg)
            } else OffRouteDecision.Hold
        } else {
            if (distM < config.deviationThresholdM * 0.5) offRouteStreak = 0
            OffRouteDecision.Hold
        }
    }

    companion object {
        /** Smallest absolute difference between two headings, degrees in [0, 180]. */
        fun angleDiffDeg(a: Double, b: Double): Double {
            var d = (a - b) % 360.0
            if (d > 180.0) d -= 360.0
            if (d < -180.0) d += 360.0
            return abs(d)
        }

        /**
         * Cross-track distance (m) from [lat]/[lon] to the nearest route segment,
         * plus the bearing (deg, 0 = North) of that segment. Equirectangular
         * projection around the query point — accurate at road-segment scale.
         */
        fun nearestDistanceAndBearing(
            lat: Double,
            lon: Double,
            route: List<Pair<Double, Double>>
        ): Pair<Double, Double> {
            val kx = 111_320.0 * cos(Math.toRadians(lat))
            fun ex(p: Pair<Double, Double>) = (p.second - lon) * kx
            fun ny(p: Pair<Double, Double>) = (p.first - lat) * 110_540.0
            var best = Double.POSITIVE_INFINITY
            var bestBearing = 0.0
            for (i in 0 until route.size - 1) {
                val ax = ex(route[i]); val ay = ny(route[i])
                val bx = ex(route[i + 1]); val by = ny(route[i + 1])
                val dx = bx - ax; val dy = by - ay
                val l2 = dx * dx + dy * dy
                // Vehicle position is the local origin (0, 0).
                val t = if (l2 < 1e-9) 0.0 else (-(ax * dx + ay * dy) / l2).coerceIn(0.0, 1.0)
                val qx = ax + t * dx; val qy = ay + t * dy
                val d = hypot(qx, qy)
                if (d < best) {
                    best = d
                    bestBearing = (Math.toDegrees(kotlin.math.atan2(dx, dy)) + 360.0) % 360.0
                }
            }
            return best to bestBearing
        }
    }
}
