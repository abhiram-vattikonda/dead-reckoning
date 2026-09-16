package com.sih.idr.navigation

import android.content.Context
import org.json.JSONArray
import org.json.JSONObject

/** Persists only the selected route geometry; old route data is replaced atomically. */
class RouteCache(context: Context) {
    private val file = java.io.File(context.filesDir, "active_route.json")

    fun save(start: Pair<Double, Double>, destination: Pair<Double, Double>, points: List<Pair<Double, Double>>, source: String) {
        val root = JSONObject().put("start", JSONArray(listOf(start.first, start.second)))
            .put("destination", JSONArray(listOf(destination.first, destination.second)))
            .put("source", source).put("timestamp", System.currentTimeMillis())
        root.put("points", JSONArray().apply { points.forEach { put(JSONArray(listOf(it.first, it.second))) } })
        val tmp = java.io.File(file.parentFile, "active_route.json.tmp")
        tmp.writeText(root.toString())
        if (!tmp.renameTo(file)) { file.delete(); tmp.renameTo(file) }
    }

    fun load(start: Pair<Double, Double>, destination: Pair<Double, Double>): List<Pair<Double, Double>>? = try {
        if (!file.exists()) return null
        val root = JSONObject(file.readText())
        val s = root.getJSONArray("start")
        val d = root.getJSONArray("destination")
        // A route from a previous trip is useful only when its origin is nearby.
        if (kotlin.math.abs(s.getDouble(0) - start.first) > 0.01 || kotlin.math.abs(s.getDouble(1) - start.second) > 0.01) return null
        if (kotlin.math.abs(d.getDouble(0) - destination.first) > 1e-6 || kotlin.math.abs(d.getDouble(1) - destination.second) > 1e-6) return null
        val arr = root.getJSONArray("points")
        buildList(arr.length()) { for (i in 0 until arr.length()) { val p = arr.getJSONArray(i); add(p.getDouble(0) to p.getDouble(1)) } }
    } catch (_: Exception) { null }

    fun clear() { file.delete() }
}
