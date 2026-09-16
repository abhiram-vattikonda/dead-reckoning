package com.sih.idr.evaluation

import com.sih.idr.data.SessionStats
import com.sih.idr.data.TrajectoryPoint
import com.sih.idr.utils.GeoUtils

/**
 * Compares the independent GPS (ground truth) and DR trajectories.
 * All errors use haversine meters. DR points are matched to each GPS fix by
 * nearest timestamp (2 s gate); unmatched fixes are skipped.
 */
object EvaluationManager {

    private const val MATCH_GATE_MILLIS = 2000L

    fun errorAt(gps: TrajectoryPoint, dr: List<TrajectoryPoint>): Double {
        if (dr.isEmpty()) return Double.NaN
        var best: TrajectoryPoint? = null
        var bestDt = Long.MAX_VALUE
        for (p in dr) {
            val dt = kotlin.math.abs(p.timestampMillis - gps.timestampMillis)
            if (dt < bestDt) { bestDt = dt; best = p }
        }
        if (best == null || bestDt > MATCH_GATE_MILLIS) return Double.NaN
        return GeoUtils.haversineMeters(gps.latitude, gps.longitude, best.latitude, best.longitude)
    }

    fun pathDistanceMeters(traj: List<TrajectoryPoint>): Double {
        var d = 0.0
        for (i in 1 until traj.size) {
            d += GeoUtils.haversineMeters(
                traj[i - 1].latitude, traj[i - 1].longitude,
                traj[i].latitude, traj[i].longitude
            )
        }
        return d
    }

    /** Live stats during a run (final/mean computed properly at STOP). */
    fun liveStats(gps: List<TrajectoryPoint>, dr: List<TrajectoryPoint>): SessionStats {
        if (gps.isEmpty() || dr.isEmpty()) return SessionStats()
        val errors = gps.mapNotNull { g ->
            errorAt(g, dr).takeIf { !it.isNaN() }
        }
        if (errors.isEmpty()) return SessionStats()
        val dist = pathDistanceMeters(gps)
        val current = errors.last()
        return SessionStats(
            currentErrorM = current,
            distanceTravelledM = dist,
            maxErrorM = errors.max(),
            meanErrorM = errors.average(),
            finalErrorM = current,
            driftPct = GeoUtils.driftPercent(current, dist)
        )
    }

    /** Final stats at STOP. */
    fun finalStats(gps: List<TrajectoryPoint>, dr: List<TrajectoryPoint>): SessionStats =
        liveStats(gps, dr)
}
