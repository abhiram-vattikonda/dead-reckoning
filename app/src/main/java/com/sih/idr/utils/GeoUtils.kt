package com.sih.idr.utils

import kotlin.math.*

/** Small, dependency-free geographic helpers. All distances in meters. */
object GeoUtils {
    private const val EARTH_RADIUS_M = 6_371_000.0

    /** Great-circle distance. NEVER subtract lat/lon degrees directly. */
    fun haversineMeters(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2).pow(2.0) +
            cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon / 2).pow(2.0)
        return 2.0 * EARTH_RADIUS_M * asin(sqrt(a.coerceIn(0.0, 1.0)))
    }

    /**
     * Destination point from [latDeg],[lonDeg] moved [northMeters] north and
     * [eastMeters] east (equirectangular approximation — fine for DR steps of cm/m).
     */
    fun moveEnu(latDeg: Double, lonDeg: Double, northMeters: Double, eastMeters: Double): Pair<Double, Double> {
        val dLat = Math.toDegrees(northMeters / EARTH_RADIUS_M)
        val dLon = Math.toDegrees(eastMeters / (EARTH_RADIUS_M * cos(Math.toRadians(latDeg))))
        return Pair(latDeg + dLat, lonDeg + dLon)
    }

    /** 0..360 degrees clockwise from North. */
    fun normalizeHeading(deg: Float): Float {
        var h = deg % 360f
        if (h < 0) h += 360f
        return h
    }

    /** drift% = position_error / total_distance * 100. NaN when distance is ~0. */
    fun driftPercent(errorM: Double, distanceM: Double): Double {
        if (distanceM.isNaN() || distanceM < 1e-6) return Double.NaN
        if (errorM.isNaN()) return Double.NaN
        return errorM / distanceM * 100.0
    }

    /** Smallest signed angular difference (target - current) wrapped to [-180, 180] degrees. */
    fun angleDiffDeg(targetDeg: Double, currentDeg: Double): Double {
        var d = (targetDeg - currentDeg) % 360.0
        if (d > 180.0) d -= 360.0
        if (d < -180.0) d += 360.0
        return d
    }

    /**
     * Geodetic (lat/lon) → local ENU meters around ([refLat],[refLon]).
     * Equirectangular approximation — accurate at road-segment scale; single
     * canonical helper so engines, tests and map matching share one frame.
     */
    fun latLonToEnu(lat: Double, lon: Double, refLat: Double, refLon: Double): Pair<Double, Double> {
        val e = Math.toRadians(lon - refLon) * EARTH_RADIUS_M * cos(Math.toRadians(refLat))
        val n = Math.toRadians(lat - refLat) * EARTH_RADIUS_M
        return e to n
    }

    /**
     * Verbose alias kept for test/call-site readability: geodetic (+alt, ignored
     * at road scale) → ENU around the reference fix.
     */
    fun geodeticToEnu(
        lat: Double, lon: Double, @Suppress("UNUSED_PARAMETER") alt: Double,
        refLat: Double, refLon: Double, @Suppress("UNUSED_PARAMETER") refAlt: Double
    ): EnuPoint = latLonToEnu(lat, lon, refLat, refLon).let { EnuPoint(it.first, it.second) }

    /** Compass bearing of the ENU direction (east, north): 0 = North, clockwise. */
    fun bearingFromEnu(east: Double, north: Double): Double =
        (Math.toDegrees(atan2(east, north)) + 360.0) % 360.0
}

/** Local ENU point in meters (x = East, y = North) around a reference fix. */
data class EnuPoint(val x: Double, val y: Double)
