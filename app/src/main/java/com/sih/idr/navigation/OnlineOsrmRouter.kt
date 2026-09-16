package com.sih.idr.navigation

import org.json.JSONObject
import java.net.HttpURLConnection
import java.net.URL

/** Small OSRM client. The response is only the active route geometry, not a raw road graph. */
object OnlineOsrmRouter {
    data class Result(val points: List<Pair<Double, Double>>, val distanceMeters: Double)

    /**
     * @param headingDeg optional compass heading of travel. When supplied it is
     * sent as an OSRM `bearings` hint (±45° tolerance) so the snapped start —
     * and therefore the first instruction — follows the direction of travel
     * instead of doubling back. Null responses (e.g. no road within tolerance)
     * fall back to offline routing at the call site.
     */
    fun route(
        startLat: Double, startLon: Double,
        endLat: Double, endLon: Double,
        headingDeg: Double? = null
    ): Result? {
        val bearings = if (headingDeg != null && headingDeg.isFinite()) {
            val h = ((headingDeg % 360.0) + 360.0) % 360.0
            "&bearings=${h.toInt()},45"
        } else ""
        val url = URL("https://router.project-osrm.org/route/v1/driving/" +
            "$startLon,$startLat;$endLon,$endLat?overview=full&geometries=geojson&steps=false$bearings")
        val connection = (url.openConnection() as HttpURLConnection).apply {
            requestMethod = "GET"
            connectTimeout = 8_000
            readTimeout = 12_000
            setRequestProperty("User-Agent", "IDRVehicleNavigator/1.0")
        }
        return try {
            if (connection.responseCode !in 200..299) return null
            val root = JSONObject(connection.inputStream.bufferedReader().use { it.readText() })
            if (root.optString("code") != "Ok") return null
            val routes = root.optJSONArray("routes") ?: return null
            if (routes.length() == 0) return null
            val route = routes.getJSONObject(0)
            val coordinates = route.getJSONObject("geometry").getJSONArray("coordinates")
            val points = buildList(coordinates.length()) {
                for (i in 0 until coordinates.length()) {
                    val pair = coordinates.getJSONArray(i)
                    add(pair.getDouble(1) to pair.getDouble(0))
                }
            }
            if (points.size < 2) null else Result(points, route.optDouble("distance", 0.0))
        } finally {
            connection.disconnect()
        }
    }
}
