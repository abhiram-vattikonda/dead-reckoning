package com.sih.idr.ui

import android.Manifest
import android.app.Application
import android.content.Intent
import android.content.pm.PackageManager
import android.location.LocationManager
import android.net.ConnectivityManager
import android.net.NetworkCapabilities
import android.os.Build
import androidx.core.content.ContextCompat
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.sih.idr.data.CsvExporter
import com.sih.idr.data.GnssMeasurement
import com.sih.idr.data.GpsSample
import com.sih.idr.data.NavigationState
import com.sih.idr.data.SessionStats
import com.sih.idr.data.TestSession
import com.sih.idr.data.TrajectoryPoint
import com.sih.idr.evaluation.EvaluationManager
import com.sih.idr.navigation.DeadReckoningEngine
import com.sih.idr.navigation.EngineDebugReporter
import com.sih.idr.navigation.GitHubStylePdrEngine
import com.sih.idr.navigation.OffRouteDecision
import com.sih.idr.navigation.OffRouteDetector
import com.sih.idr.navigation.pdr.FixedStrideLength
import com.sih.idr.navigation.pdr.PdrConfig
import com.sih.idr.navigation.OnlineOsrmRouter
import com.sih.idr.navigation.RouteCache
import com.sih.idr.sensors.GpsManager
import com.sih.idr.sensors.ImuManager
import com.sih.idr.sensors.OrientationManager
import com.sih.idr.sensors.RecordingService
import com.sih.idr.utils.GeoUtils
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.Dispatchers
import java.io.File

/** Throttled (~5 Hz) snapshot for the Compose UI. */
data class UiSnapshot(
    val lastGps: GpsSample? = null,
    val lastDr: NavigationState? = null,
    val gpsTrajectory: List<TrajectoryPoint> = emptyList(),
    val drTrajectory: List<TrajectoryPoint> = emptyList(),
    val stats: SessionStats = SessionStats(),
    val imuCount: Long = 0L,
    /** Live measured IMU callback rate (Hz) — independent of GPS/location state. */
    val imuHz: Float = 0f,
    val gpsCount: Int = 0,
    /** DR engine outputs produced — the liveness discriminant vs IMU count. */
    val drStateCount: Int = 0,
    val stepCount: Int = 0,
    /** Phone master location switch (GPS on/off), for the "GPS off" hint. */
    val locationOn: Boolean = true
    ,val plannedRoute: List<TrajectoryPoint> = emptyList()
    ,val engineHealth: String = "idle"
)

class TrackingViewModel(app: Application) : AndroidViewModel(app) {

    // ------------------------------------------------------------------ layers
    val orientationManager = OrientationManager(app.applicationContext)
    val imuManager = ImuManager(app.applicationContext, orientationManager)
    val gpsManager = GpsManager(app.applicationContext)

    /**
     * >>> SINGLE SWAP SITE FOR ENGINES <<<
     * Current: GitHubStylePdrEngine (pedestrian baseline ported from
     * nisargnp/DeadReckoning). Future: MMMDeadReckoningEngine — same
     * DeadReckoningEngine interface, nothing else in the app changes.
     * (BaselineDeadReckoningEngine.kt is retained in the repo but inactive.)
     */
    private val pdrConfig = PdrConfig()
    private val vehicleEngine = com.sih.idr.navigation.RouteAwareFusionEngine()
    private val roadGraph = com.sih.idr.navigation.OfflineRoadGraphCatalog.fromAssets(app.applicationContext, listOf("road_graph_guntur.json", "road_graph_vijayawada.json"))
    private val routeCache = RouteCache(app.applicationContext)
    private val engine: DeadReckoningEngine = vehicleEngine

    /** Shown in the UI so a vehicle test is never mistaken for the final model. */
    val engineLabel: String =
        "Vehicle route-aware fusion · IMU + GNSS + route constraint"

    // ------------------------------------------------------------------ switches
    /** Testing Mode ON: GPS runs continuously as independent ground truth. */
    val testingMode = MutableStateFlow(true)

    /**
     * Navigation Engine switch. Dead reckoning runs ONLY while this is ON.
     * Turning it ON snapshots the latest known GPS fix as the DR seed
     * (whatever its age — the age is reported); with no fix ever seen, the
     * engine starts a clearly-labeled relative track instead.
     * Turning it OFF freezes the red trail. GPS logging is unaffected either way.
     *
     * GNSS fixes are additionally forwarded via
     * [DeadReckoningEngine.onGnssMeasurement] while the engine runs
     * (extension point for future fusion; the baseline ignores them).
     */
    val navEngineEnabled = MutableStateFlow(false)

    /** Human-readable DR seed description for the UI. */
    val seedInfo = MutableStateFlow("DR seed: — (engine off)")

    /**
     * True when DR was seeded from a real GPS fix (absolute trail).
     * False = relative track from null origin: marker/follow are hidden so the
     * map never flies to the ocean; the shape is still recorded (see CSV x/y).
     */
    val hasGpsSeed = MutableStateFlow(false)

    /**
     * Automatic off-route recalculation. When ON and a destination is set, the
     * [OffRouteDetector] watches the vehicle pose (fresh GPS preferred, DR
     * otherwise — works with no GPS and no network) and rebuilds the route from
     * the current position when a wrong turn is confirmed.
     */
    val autoRecalculate = MutableStateFlow(true)

    /** Status line for the last (re)calculation, shown under the route controls. */
    val recalcInfo = MutableStateFlow("")

    // ------------------------------------------------------------------ state
    private val _isRecording = MutableStateFlow(false)
    val isRecording: StateFlow<Boolean> = _isRecording.asStateFlow()

    private val _drStatus = MutableStateFlow("DR: idle")
    val drStatus: StateFlow<String> = _drStatus.asStateFlow()

    private val _snapshot = MutableStateFlow(UiSnapshot())
    val snapshot: StateFlow<UiSnapshot> = _snapshot.asStateFlow()

    private val _messages = MutableSharedFlow<String>(extraBufferCapacity = 8)
    val messages: SharedFlow<String> = _messages.asSharedFlow()

    private val _exportedFiles = MutableStateFlow<List<File>>(emptyList())
    val exportedFiles: StateFlow<List<File>> = _exportedFiles.asStateFlow()

    private var session: TestSession? = null
    private var lastUiPushMs = 0L
    private var lastDrPointMs = 0L
    private var lastPdrLogMs = 0L
    private var destination: Pair<Double, Double>? = null
    private var plannedRouteLatLon: List<Pair<Double, Double>> = emptyList()
    private val offRouteDetector = OffRouteDetector()
    private var recalculating = false

    companion object {
        /** DR-only route rebuilds above this uncertainty do more harm than good. */
        private const val DR_RECALC_MAX_CONF_M = 30f
    }

    init {
        orientationManager.start()
        imuManager.listener = ::onImuSample
        gpsManager.listener = ::onGpsSample
        // Destination-less map matching: when no route is planned the engine
        // snaps onto the nearest road in the direction of travel (offline
        // graphs, IMU thread, throttled inside the engine).
        vehicleEngine.roadSnapper =
            com.sih.idr.navigation.RoadSnapProvider { lat, lon, headingDeg ->
                roadGraph.snapToRoad(lat, lon, headingDeg)
            }
        // Testing Mode defaults ON -> GPS runs continuously from launch,
        // but ONLY if permission is already granted (else wait for the grant callback).
        startGpsIfPermitted()
    }

    // ------------------------------------------------------------- permissions
    private fun hasLocationPermission(): Boolean {
        val ctx = getApplication<Application>()
        return ContextCompat.checkSelfPermission(
            ctx, Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED ||
            ContextCompat.checkSelfPermission(
                ctx, Manifest.permission.ACCESS_COARSE_LOCATION
            ) == PackageManager.PERMISSION_GRANTED
    }

    private fun startGpsIfPermitted() {
        if (hasLocationPermission() && !gpsManager.isRunning()) gpsManager.start()
    }

    /** Called by MainActivity once the user grants location permission. */
    fun onLocationPermissionResult(granted: Boolean) {
        if (granted) {
            startGpsIfPermitted()
            pushSnapshot(force = true)
        } else {
            _messages.tryEmit("Location permission denied — GPS and START are unavailable")
        }
    }

    // ------------------------------------------------------------- public API
    fun setTestingMode(on: Boolean) {
        testingMode.value = on
        if (on) {
            startGpsIfPermitted()
            _messages.tryEmit("Testing Mode ON: GPS runs as independent ground truth")
        } else {
            if (!_isRecording.value) gpsManager.stop()
            _messages.tryEmit("Testing Mode OFF")
        }
    }

    fun setNavEngineEnabled(on: Boolean) {
        if (on && !_isRecording.value) {
            _messages.tryEmit("Press START first — then enable the navigation engine")
            return
        }
        if (on == navEngineEnabled.value) return
        if (on) enableNavEngine() else disableNavEngine()
    }

    fun setDestination(latitude: Double, longitude: Double) {
        if (latitude !in -90.0..90.0 || longitude !in -180.0..180.0) {
            _messages.tryEmit("Invalid destination coordinates")
            return
        }
        destination = latitude to longitude
        offRouteDetector.reset()
        recalcInfo.value = ""
        val fix = gpsManager.lastFix.value
        if (fix != null) {
            // Immediate visual preview. It is replaced by a road-following route asynchronously.
            plannedRouteLatLon = listOf(fix.latitude to fix.longitude, latitude to longitude)
            vehicleEngine.setDestination(latitude, longitude)
            pushSnapshot(force = true)
            requestRoadRoute(fix.latitude, fix.longitude, latitude, longitude, fix.bearing?.toDouble())
        } else {
            vehicleEngine.setDestination(latitude, longitude)
            plannedRouteLatLon = emptyList()
            _messages.tryEmit("Waiting for GPS to select the Guntur/Vijayawada road graph")
        }
        pushSnapshot(force = true)
        _messages.tryEmit("Destination set; route constraint enabled")
    }

    fun clearDestination() { destination = null; plannedRouteLatLon = emptyList(); routeCache.clear(); vehicleEngine.clearRoute(); offRouteDetector.reset(); recalcInfo.value = ""; pushSnapshot(force = true) }

    fun setAutoRecalculate(on: Boolean) {
        autoRecalculate.value = on
        if (on) offRouteDetector.reset()
        _messages.tryEmit(if (on) "Auto-recalculate ON — wrong turns rebuild the route" else "Auto-recalculate OFF")
    }

    private fun requestRoadRoute(startLat: Double, startLon: Double, endLat: Double, endLon: Double, headingDeg: Double? = null) {
        val destinationKey = endLat to endLon
        viewModelScope.launch(Dispatchers.IO) {
            val online = runCatching { OnlineOsrmRouter.route(startLat, startLon, endLat, endLon, headingDeg) }.getOrNull()
            val route = online?.points ?: roadGraph.routeLatLon(startLat, startLon, endLat, endLon, headingDeg)
            val source = if (online != null) "OSRM online road route" else "bundled offline road graph"
            kotlinx.coroutines.withContext(Dispatchers.Main) {
                if (destination != destinationKey) return@withContext
                if (route.size >= 2) {
                    plannedRouteLatLon = route
                    vehicleEngine.setRouteLatLon(route)
                    routeCache.save(startLat to startLon, destinationKey, route, source)
                    _messages.tryEmit("$source loaded: ${route.size} points${if (online == null && roadGraph.lastUsedNearestRoadFallback) " (nearest reachable road fallback)" else ""}")
                } else {
                    _messages.tryEmit("No connected road route found; keeping straight preview")
                }
                pushSnapshot(force = true)
            }
        }
    }

    // ------------------------------------------------- off-route recalculation
    private data class NavPose(
        val lat: Double,
        val lon: Double,
        val headingDeg: Double,
        val speedMps: Double,
        val source: String // "GPS" or "DR" — shown so tests stay honest
    )

    /**
     * Vehicle pose for route monitoring. Prefers a fresh GPS fix (< 8 s old);
     * otherwise falls back to the DR estimate — this is what makes wrong-turn
     * detection and recalculation work with phone location OFF and no network.
     * Heading/speed come from the IMU-driven engine whenever it runs.
     */
    private fun currentPose(): NavPose? {
        val now = System.currentTimeMillis()
        val fix = gpsManager.lastFix.value
        val freshGps = fix?.takeIf { now - it.timestampMillis < 8000 }
        val dr = if (navEngineEnabled.value) engine.currentState else null
        if (freshGps == null && dr == null) return null
        val heading = dr?.headingDeg?.toDouble()
            ?: freshGps?.bearing?.toDouble()
            ?: return null // no heading source: cannot judge turns
        val speed = dr?.speed?.toDouble()
            ?: freshGps?.speed?.toDouble()
            ?: 0.0
        val lat = freshGps?.latitude ?: dr!!.latitude
        val lon = freshGps?.longitude ?: dr!!.longitude
        return NavPose(lat, lon, heading, speed, if (freshGps != null) "GPS" else "DR")
    }

    /** Called on IMU and GPS updates; the detector itself throttles to ~0.5 Hz. */
    private fun maybeCheckRoute() {
        if (!_isRecording.value || !navEngineEnabled.value || !autoRecalculate.value) return
        val dest = destination ?: return
        if (plannedRouteLatLon.size < 2 || recalculating) return
        val pose = currentPose() ?: return
        // Guard: rebuilding the road network from a drifted DR-only pose bakes
        // the drift into the new route and pins the trail to the wrong road.
        // Hold (free inertial coast) until GPS returns or confidence recovers.
        if (pose.source == "DR") {
            val conf = vehicleEngine.currentState?.confidenceMeters ?: Float.MAX_VALUE
            if (!conf.isFinite() || conf > DR_RECALC_MAX_CONF_M) return
        }
        when (val d = offRouteDetector.update(
            System.currentTimeMillis(), pose.lat, pose.lon,
            pose.headingDeg, pose.speedMps, plannedRouteLatLon
        )) {
            is OffRouteDecision.Recalculate -> recalculateRoute(pose, dest, d)
            OffRouteDecision.Hold -> Unit
        }
    }

    /**
     * Rebuilds the route from the current pose to the same destination.
     * The pose heading is passed to both routers so the new route continues
     * FORWARD (no U-turn instruction): OSRM gets it as a bearings hint, the
     * offline graph snaps to the road ahead. Offline-first: bundled road graph
     * (works with no WiFi); OSRM online is attempted only when a network is
     * actually available. Never blocks sensors — Dijkstra runs on Dispatchers.IO.
     */
    private fun recalculateRoute(pose: NavPose, dest: Pair<Double, Double>, decision: OffRouteDecision.Recalculate) {
        recalculating = true
        recalcInfo.value = "Off route (%.0fm via %s) — recalculating…".format(decision.deviationM, pose.source)
        viewModelScope.launch(Dispatchers.IO) {
            val online = if (isNetworkAvailable()) {
                runCatching { OnlineOsrmRouter.route(pose.lat, pose.lon, dest.first, dest.second, pose.headingDeg) }.getOrNull()
            } else null
            val route = online?.points ?: roadGraph.routeLatLon(pose.lat, pose.lon, dest.first, dest.second, pose.headingDeg)
            val source = if (online != null) "OSRM online" else "offline road graph"
            kotlinx.coroutines.withContext(kotlinx.coroutines.Dispatchers.Main) {
                recalculating = false
                if (destination != dest) return@withContext // destination changed meanwhile
                if (route.size >= 2) {
                    plannedRouteLatLon = route
                    vehicleEngine.setRouteLatLon(route)
                    routeCache.save(pose.lat to pose.lon, dest, route, "auto-recalc $source")
                    recalcInfo.value = "Route recalculated ${offRouteDetector.recalculations}× · $source" +
                        " · off-route %.0fm, turn %.0f° (pose: %s)".format(
                            decision.deviationM, decision.turnDeg, pose.source)
                    _messages.tryEmit("New route from current position (${route.size} points, $source)")
                } else {
                    recalcInfo.value = "Recalculation found no road — keeping old route"
                    _messages.tryEmit("No connected road from here; keeping old route")
                }
                pushSnapshot(force = true)
            }
        }
    }

    private fun isNetworkAvailable(): Boolean {
        val cm = getApplication<Application>().getSystemService(ConnectivityManager::class.java)
            ?: return false
        return try {
            val net = cm.activeNetwork ?: return false
            val caps = cm.getNetworkCapabilities(net) ?: return false
            caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) ||
                caps.hasTransport(NetworkCapabilities.TRANSPORT_CELLULAR) ||
                caps.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET)
        } catch (_: Exception) {
            false
        }
    }
    /** Seed DR from the latest known GPS fix and start IMU-only navigation. */
    private fun enableNavEngine() {
        val fix = gpsManager.lastFix.value
        engine.reset()
        lastDrPointMs = 0L
        if (fix != null) {
            val speed = fix.speed ?: 0f
            val heading = fix.bearing ?: orientationManager.currentYawDeg() ?: 0f
            val headingRad = Math.toRadians(heading.toDouble())
            // >>> THE BOUNDARY: after this call the engine sees IMU only
            // (plus onGnssMeasurement forwarding while enabled — ignored by baseline).
            engine.initialize(
                NavigationState(
                    timestampNanos = System.nanoTime(),
                    latitude = fix.latitude,
                    longitude = fix.longitude,
                    altitude = fix.altitude ?: 0.0,
                    velEast = (speed * Math.sin(headingRad)).toFloat(),
                    velNorth = (speed * Math.cos(headingRad)).toFloat(),
                    headingDeg = GeoUtils.normalizeHeading(heading)
                )
            )
            destination?.let { (lat, lon) ->
                val cached = routeCache.load(fix.latitude to fix.longitude, lat to lon)
                if (cached != null) {
                    plannedRouteLatLon = cached
                    vehicleEngine.setRouteLatLon(cached)
                    _messages.tryEmit("Cached road route restored: ${cached.size} points")
                } else {
                    plannedRouteLatLon = listOf(fix.latitude to fix.longitude, lat to lon)
                    vehicleEngine.setDestination(lat, lon)
                    requestRoadRoute(fix.latitude, fix.longitude, lat, lon, fix.bearing?.toDouble())
                }
            }
            val ageS = (System.currentTimeMillis() - fix.timestampMillis) / 1000
            seedInfo.value =
                "DR seed: GPS fix, age ${ageS}s @ (%.5f, %.5f)".format(fix.latitude, fix.longitude)
            hasGpsSeed.value = true
            _messages.tryEmit("Navigation engine ON — seeded from GPS, now IMU-only")
        } else {
            // No GPS ever seen (e.g. phone location off since boot):
            // run a relative track, clearly labeled as having no absolute position.
            val startYaw = orientationManager.currentYawDeg() ?: 0f
            engine.initialize(
                NavigationState(timestampNanos = System.nanoTime(), latitude = 0.0, longitude = 0.0, headingDeg = startYaw)
            )
            seedInfo.value = "DR seed: NONE — relative track (no absolute position)"
            hasGpsSeed.value = false
            _messages.tryEmit("Navigation engine ON: no GPS seed, recording relative track")
        }
        navEngineEnabled.value = true
        offRouteDetector.reset()
        refreshDrStatus()
        pushSnapshot(force = true)
    }

    private fun disableNavEngine() {
        navEngineEnabled.value = false
        offRouteDetector.reset()
        recalculating = false
        hasGpsSeed.value = false
        seedInfo.value = "DR seed: — (engine off)"
        _messages.tryEmit("Navigation engine OFF — red trail frozen, GPS logging continues")
        refreshDrStatus()
        pushSnapshot(force = true)
    }

    /** Phone master location switch (user can turn GPS off anytime — supported). */
    private fun isLocationEnabled(): Boolean {
        val lm = getApplication<Application>().getSystemService(LocationManager::class.java)
            ?: return true
        return try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) lm.isLocationEnabled
            else lm.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
                lm.isProviderEnabled(LocationManager.NETWORK_PROVIDER)
        } catch (_: Exception) {
            true
        }
    }

    /**
     * START begins the test session (IMU + GPS logging, blue trail).
     * NEVER blocked by GPS: works with phone location OFF — the blue trail
     * simply has no data until fixes arrive. Does NOT start DR (see engine switch).
     */
    fun onStart() {
        if (_isRecording.value) return
        beginSession()
    }

    fun onStop() {
        if (navEngineEnabled.value) disableNavEngine()
        if (!_isRecording.value) return
        imuManager.stop()
        try {
            getApplication<Application>().stopService(
                Intent(getApplication(), RecordingService::class.java)
            )
        } catch (_: Exception) { }
        val s = session
        if (s != null) {
            s.endTimestampMillis = System.currentTimeMillis()
            s.stats = EvaluationManager.finalStats(s.gpsTrajectory, s.drTrajectory)
        }
        _isRecording.value = false
        _drStatus.value = "DR: stopped"
        if (!testingMode.value) gpsManager.stop()
        pushSnapshot(force = true)
        val st = s?.stats
        _messages.tryEmit(
            if (st != null && !st.finalErrorM.isNaN())
                "Session stopped: final error %.1f m, drift %.2f%%".format(st.finalErrorM, st.driftPct)
            else "Session stopped"
        )
    }

    fun onReset() {
        onStop()
        engine.reset()
        session = null
        lastDrPointMs = 0L
        lastPdrLogMs = 0L
        offRouteDetector.reset()
        recalculating = false
        recalcInfo.value = ""
        _snapshot.value = UiSnapshot()
        _exportedFiles.value = emptyList()
        seedInfo.value = "DR seed: — (engine off)"
        hasGpsSeed.value = false
        _drStatus.value = "DR: idle"
        _messages.tryEmit("Session cleared")
    }

    fun exportCsv() {
        val s = session
        if (s == null || (s.gpsSamples.isEmpty() && s.imuSamples.isEmpty())) {
            _messages.tryEmit("Nothing to export yet")
            return
        }
        viewModelScope.launch {
            try {
                val result: CsvExporter.ExportResult =
                    CsvExporter.exportSession(getApplication(), s, s.pdrDebug)
                _exportedFiles.value = listOf(result.navCsv, result.imuCsv, result.pdrCsv)
                _messages.tryEmit("Exported:\n${result.navCsv.absolutePath}\n${result.imuCsv.absolutePath}\n${result.pdrCsv.absolutePath}")
            } catch (e: Exception) {
                _messages.tryEmit("Export failed: ${e.message}")
            }
        }
    }

    // ------------------------------------------------------------- internals
    private fun beginSession() {
        session = TestSession(startTimestampMillis = System.currentTimeMillis())
        // Log the current fix as the first blue point if we have one — no requirement.
        gpsManager.lastFix.value?.let { fix ->
            session?.gpsSamples?.add(fix)
            session?.gpsTrajectory?.add(TrajectoryPoint(fix.timestampMillis, fix.latitude, fix.longitude))
        }
        imuManager.start()
        startGpsIfPermitted()
        // Foreground priority: keeps sensor/GPS delivery alive with screen off
        // or app backgrounded. Sensors do NOT depend on the location switch.
        try {
            ContextCompat.startForegroundService(
                getApplication(),
                Intent(getApplication(), RecordingService::class.java)
            )
        } catch (_: Exception) { }
        _isRecording.value = true
        refreshDrStatus()
        _messages.tryEmit(
            if (isLocationEnabled()) "Session started — enable the navigation engine when ready"
            else "Session started with phone GPS OFF — logging IMU; blue trail resumes when GPS returns"
        )
        pushSnapshot(force = true)
    }

    /** Runs on the imu-thread (NOT the UI thread). Must stay allocation-light. */
    private fun onImuSample(sample: com.sih.idr.data.ImuSample) {
        if (!_isRecording.value) return
        val s = session ?: return
        s.imuSamples.add(sample) // IMU always logged during a session
        if (!navEngineEnabled.value) {
            // Navigation engine off: no DR processing at all.
            val wall = System.currentTimeMillis()
            if (wall - lastUiPushMs >= 200L) pushSnapshot()
            return
        }
        val nav = engine.processImu(sample)
        s.drStates.add(nav)
        // Engine intermediates: every step + ~10 Hz track (for CSV/ML debugging).
        // Via the EngineDebugReporter interface so ANY active engine is logged.
        (engine as? EngineDebugReporter)?.lastDebug?.let { dbg ->
            val nowMsDbg = System.currentTimeMillis()
            if (dbg.stepDetected || nowMsDbg - lastPdrLogMs >= 100L) {
                lastPdrLogMs = nowMsDbg
                s.pdrDebug.add(dbg)
            }
        }
        val nowMs = sample.timestampNanos / 1_000_000
        if (nowMs - lastDrPointMs >= 100L) { // 10 Hz nav output decimation
            lastDrPointMs = nowMs
            s.drTrajectory.add(TrajectoryPoint(System.currentTimeMillis(), nav.latitude, nav.longitude))
        }
        maybeCheckRoute() // off-route detection runs on the DR (IMU) stream: no GPS needed
        val wall = System.currentTimeMillis()
        if (wall - lastUiPushMs >= 200L) pushSnapshot()
    }

    /** Runs on the main looper. GPS truth path — independent of the engine. */
    private fun onGpsSample(sample: GpsSample) {
        if (!_isRecording.value) {
            // Not recording (Testing Mode idle): still refresh the map position.
            pushSnapshot(force = false)
            return
        }
        val s = session ?: return
        s.gpsSamples.add(sample)
        s.gpsTrajectory.add(TrajectoryPoint(sample.timestampMillis, sample.latitude, sample.longitude))
        if (plannedRouteLatLon.isEmpty()) {
            destination?.let { (lat, lon) ->
                roadGraph.routeLatLon(sample.latitude, sample.longitude, lat, lon, sample.bearing?.toDouble()).let { route ->
                    if (route.size >= 2) { plannedRouteLatLon = route; vehicleEngine.setRouteLatLon(route); _messages.tryEmit(if (roadGraph.lastUsedNearestRoadFallback) "Destination disconnected: path starts on nearest reachable road" else "Offline OSM road route calculated: ${route.size} points"); pushSnapshot(force = true) }
                }
            }
        }
        // GNSS -> engine ONLY while the engine runs (baseline ignores it by design).
        if (navEngineEnabled.value) {
            try { engine.onGnssMeasurement(GnssMeasurement(sample)) } catch (_: Exception) { }
        }
        maybeCheckRoute()
        pushSnapshot()
    }

    private fun refreshDrStatus() {
        _drStatus.value = when {
            !_isRecording.value -> "DR: idle (engine off)"
            navEngineEnabled.value -> "DR: running (IMU-only)"
            else -> "DR: idle (engine off)"
        }
    }

    private fun pushSnapshot(force: Boolean = false) {
        val wall = System.currentTimeMillis()
        if (!force && wall - lastUiPushMs < 200L) return
        lastUiPushMs = wall
        val s = session
        val gpsTraj = s?.gpsTrajectory?.toList() ?: emptyList()
        val drTraj = s?.drTrajectory?.toList() ?: emptyList()
        val stats = if (s != null && gpsTraj.isNotEmpty() && drTraj.isNotEmpty()) {
            if (_isRecording.value) EvaluationManager.liveStats(gpsTraj, drTraj) else s.stats
        } else SessionStats()
        _snapshot.value = UiSnapshot(
            lastGps = gpsManager.lastFix.value,
            lastDr = if (navEngineEnabled.value) engine.currentState else null,
            gpsTrajectory = gpsTraj,
            drTrajectory = drTraj,
            stats = stats,
            imuCount = imuManager.sampleCount,
            imuHz = imuManager.measuredHz,
            gpsCount = s?.gpsSamples?.size ?: 0,
            drStateCount = s?.drStates?.size ?: 0,
            stepCount = s?.pdrDebug?.count { it.stepDetected } ?: 0,
            locationOn = isLocationEnabled()
            ,plannedRoute = plannedRouteLatLon.mapIndexed { i, p -> TrajectoryPoint(i.toLong(), p.first, p.second) }
            ,engineHealth = buildString {
                append(vehicleEngine.status)
                if (vehicleEngine.isStationary) append(" · ZUPT active")
                if (vehicleEngine.mountCalibrated) append(" · mount aligned")
                if (vehicleEngine.speedFailure) append(" · speed fault")
                if (vehicleEngine.lateralFailure) append(" · lateral fault")
            }
        )
    }

    override fun onCleared() {
        imuManager.stop()
        gpsManager.stop()
        super.onCleared()
    }
}
