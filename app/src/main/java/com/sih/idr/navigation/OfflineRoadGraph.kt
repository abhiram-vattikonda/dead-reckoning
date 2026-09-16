package com.sih.idr.navigation

import android.content.Context
import org.json.JSONObject
import java.io.InputStream
import java.util.PriorityQueue
import kotlin.math.hypot

/** Offline OSM graph in the same format produced by the Python converter:
 * {"origin":[lat,lon],"nodes":{"id":[east_m,north_m]},"edges":[{"a":id,"b":id,"oneway":"no"}]} */
data class RoadNode(val id: String, val east: Double, val north: Double)
data class RoadEdge(val to: String, val cost: Double)

class OfflineRoadGraph private constructor(
    val originLat: Double,
    val originLon: Double,
    private val nodes: Map<String, RoadNode>,
    private val adjacency: Map<String, List<RoadEdge>>
) {
    var lastUsedNearestRoadFallback: Boolean = false
        private set
    fun nearestNode(east: Double, north: Double): String = nodes.values.minBy { hypot(it.east - east, it.north - north) }.id
    fun nearestDistance(east: Double, north: Double): Double = nodes.values.minOf { hypot(it.east - east, it.north - north) }

    /** Dijkstra route. Returns ENU points including snapped start/end graph nodes. */
    fun route(startEast: Double, startNorth: Double, endEast: Double, endNorth: Double): List<Pair<Double, Double>> {
        val start = nearestNode(startEast, startNorth); val goal = nearestNode(endEast, endNorth)
        val ids = dijkstraIds(start, goal, endEast, endNorth)
            ?: return emptyList()
        return ids.map { nodes.getValue(it).east to nodes.getValue(it).north }
    }

    /**
     * Heading-aware route: prefers continuing FORWARD along the road the vehicle
     * is actually traveling instead of snapping to the nearest node (which is
     * often the road just traveled → a U-turn instruction).
     *
     * Each nearby directed edge is scored by lateral distance minus an alignment
     * bonus for pointing along [headingRad] (compass radians, 0 = North).
     * Routing then starts from the downstream node of the winning edge, so the
     * path physically cannot begin by doubling back. If every nearby option
     * points backwards (dead-end, parking pocket), falls back to plain [route] —
     * a U-turn out is legitimate there and the only way out.
     */
    fun routeWithHeading(
        startEast: Double, startNorth: Double,
        headingRad: Double,
        endEast: Double, endNorth: Double
    ): List<Pair<Double, Double>> {
        val hx = kotlin.math.sin(headingRad); val hy = kotlin.math.cos(headingRad)
        var bestDownstream: String? = null
        var bestProjEast = 0.0; var bestProjNorth = 0.0
        var bestScore = Double.POSITIVE_INFINITY
        var bestAlign = Double.NEGATIVE_INFINITY
        for ((id, node) in nodes) {
            if (hypot(node.east - startEast, node.north - startNorth) > SEARCH_RADIUS_M) continue
            for (edge in adjacency[id].orEmpty()) {
                val nb = nodes[edge.to] ?: continue
                val dx = nb.east - node.east; val dy = nb.north - node.north
                val len = hypot(dx, dy); if (len < 1e-6) continue
                val ux = dx / len; val uy = dy / len
                val t = (((startEast - node.east) * dx + (startNorth - node.north) * dy) / (len * len))
                    .coerceIn(0.0, 1.0)
                val qx = node.east + t * dx; val qy = node.north + t * dy
                val d = hypot(startEast - qx, startNorth - qy)
                if (d > SEARCH_RADIUS_M) continue
                val align = ux * hx + uy * hy // +1 along travel, -1 against it
                val score = d - ALIGN_WEIGHT_M * align
                if (score < bestScore) {
                    bestScore = score; bestAlign = align
                    bestDownstream = edge.to; bestProjEast = qx; bestProjNorth = qy
                }
            }
        }
        val downstream = bestDownstream
        if (downstream == null || bestAlign < -0.3) return route(startEast, startNorth, endEast, endNorth)
        val goal = nearestNode(endEast, endNorth)
        val ids = dijkstraIds(downstream, goal, endEast, endNorth) ?: return route(startEast, startNorth, endEast, endNorth)
        val pts = mutableListOf(bestProjEast to bestProjNorth)
        pts += ids.map { nodes.getValue(it).east to nodes.getValue(it).north }
        return pts
    }

    /** Dijkstra node-id path, or null when the goal is unreachable from start. */
    private fun dijkstraIds(start: String, goal: String, endEast: Double, endNorth: Double): List<String>? {
        val dist = mutableMapOf(start to 0.0); val prev = mutableMapOf<String, String?>(); prev[start] = null
        val q = PriorityQueue(compareBy<Pair<Double, String>> { it.first }); q.add(0.0 to start)
        while (q.isNotEmpty()) {
            val item = q.poll() ?: break
            val d = item.first; val id = item.second
            if (d > (dist[id] ?: Double.POSITIVE_INFINITY)) continue
            if (id == goal) break
            for (edge in adjacency[id].orEmpty()) {
                val nd = d + edge.cost
                if (nd < (dist[edge.to] ?: Double.POSITIVE_INFINITY)) { dist[edge.to] = nd; prev[edge.to] = id; q.add(nd to edge.to) }
            }
        }
        lastUsedNearestRoadFallback = false
        var target = goal
        if (!prev.containsKey(goal)) {
            // The destination may be in a disconnected/imported map component.
            // Keep the vehicle on the closest road reachable from its start;
            // never invent a straight segment across unmapped space.
            target = dist.keys.minByOrNull { id ->
                hypot(nodes.getValue(id).east - endEast, nodes.getValue(id).north - endNorth)
            } ?: return null
            lastUsedNearestRoadFallback = true
        }
        val ids = mutableListOf<String>(); var cur: String? = target
        while (cur != null) { ids.add(cur); cur = prev[cur] }
        return ids.asReversed()
    }

    fun routeLatLon(startLat: Double, startLon: Double, endLat: Double, endLon: Double): List<Pair<Double, Double>> =
        routeLatLon(startLat, startLon, endLat, endLon, headingDeg = null)

    /**
     * @param headingDeg optional compass heading of travel (0 = North). When
     * supplied, the route starts from the road ahead (no U-turn instruction);
     * when null, plain nearest-node snapping is used.
     */
    fun routeLatLon(
        startLat: Double, startLon: Double,
        endLat: Double, endLon: Double,
        headingDeg: Double? = null
    ): List<Pair<Double, Double>> {
        val scale = 6378137.0; val c = kotlin.math.cos(Math.toRadians(originLat))
        val se = Math.toRadians(startLon - originLon) * scale * c; val sn = Math.toRadians(startLat - originLat) * scale
        val ee = Math.toRadians(endLon - originLon) * scale * c; val en = Math.toRadians(endLat - originLat) * scale
        // Never snap coordinates from another city/country to this graph.
        // The bundled graph is local to its OSM extract; 1 km is a conservative
        // maximum distance from a graph node for route initialization.
        if (nearestDistance(se, sn) > 1000.0 || nearestDistance(ee, en) > 1000.0) return emptyList()
        val enu = if (headingDeg != null && headingDeg.isFinite()) {
            routeWithHeading(se, sn, Math.toRadians(headingDeg), ee, en)
        } else {
            route(se, sn, ee, en)
        }
        val points = enu.map { p ->
            Math.toDegrees(p.second / scale) + originLat to Math.toDegrees(p.first / (scale * c)) + originLon
        }
        // Show the actual current position before the first snapped road node.
        return if (points.isEmpty()) points else listOf(startLat to startLon) + points
    }

    companion object {
        /** Road-search radius (m) for heading-aware start selection. */
        const val SEARCH_RADIUS_M = 150.0
        /** Alignment bonus (m): a fully aligned edge wins over one 150 m closer. */
        const val ALIGN_WEIGHT_M = 150.0

        fun fromJson(input: InputStream): OfflineRoadGraph {
            val root = JSONObject(input.bufferedReader().use { it.readText() })
            val origin = root.getJSONArray("origin")
            val map = mutableMapOf<String, RoadNode>(); val jsonNodes = root.getJSONObject("nodes")
            jsonNodes.keys().forEach { id -> val p = jsonNodes.getJSONArray(id); map[id] = RoadNode(id, p.getDouble(0), p.getDouble(1)) }
            val adj = map.keys.associateWith { mutableListOf<RoadEdge>() }
            val edges = root.getJSONArray("edges")
            for (i in 0 until edges.length()) {
                val e = edges.getJSONObject(i); val a = e.getString("a"); val b = e.getString("b")
                val pa = map.getValue(a); val pb = map.getValue(b); val cost = hypot(pa.east - pb.east, pa.north - pb.north)
                adj.getValue(a).add(RoadEdge(b, cost))
                if (e.optString("oneway", "no").lowercase() !in setOf("yes", "true", "1")) adj.getValue(b).add(RoadEdge(a, cost))
            }
            return OfflineRoadGraph(origin.getDouble(0), origin.getDouble(1), map, adj)
        }
        fun fromAssets(context: Context, name: String = "road_graph.json"): OfflineRoadGraph? = try { fromJson(context.assets.open(name)) } catch (_: Exception) { null }
    }
}

class OfflineRoadGraphCatalog(private val graphs: List<OfflineRoadGraph>) {
    var lastUsedNearestRoadFallback = false
        private set

    fun routeLatLon(startLat: Double, startLon: Double, endLat: Double, endLon: Double): List<Pair<Double, Double>> =
        routeLatLon(startLat, startLon, endLat, endLon, headingDeg = null)

    fun routeLatLon(
        startLat: Double, startLon: Double,
        endLat: Double, endLon: Double,
        headingDeg: Double? = null
    ): List<Pair<Double, Double>> {
        // Select a graph that covers both endpoints; never route between cities
        // through an unrelated regional extract.
        val candidates = graphs.mapNotNull { graph ->
            val route = graph.routeLatLon(startLat, startLon, endLat, endLon, headingDeg)
            if (route.size >= 2) graph to route else null
        }
        val selected = candidates.firstOrNull()
        lastUsedNearestRoadFallback = selected?.first?.lastUsedNearestRoadFallback == true
        return selected?.second ?: emptyList()
    }

    companion object {
        fun fromAssets(context: Context, names: List<String>): OfflineRoadGraphCatalog {
            return OfflineRoadGraphCatalog(names.mapNotNull { OfflineRoadGraph.fromAssets(context, it) })
        }
    }
}
