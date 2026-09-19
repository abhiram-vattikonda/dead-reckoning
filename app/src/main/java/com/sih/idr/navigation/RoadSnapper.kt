package com.sih.idr.navigation

/**
 * Destination-less map-matching source for [RouteAwareFusionEngine].
 *
 * When no destination/route is active, the engine asks the provider for the
 * road segment the vehicle is most likely traveling (nearest + heading
 * aligned) and snaps laterally onto it — the same constraint as a planned
 * route, but discovered from the direction of travel. Implemented by the
 * app layer over [OfflineRoadGraphCatalog]; the engine only sees lat/lon.
 * Runs on the IMU thread: implementations must be pure CPU (no I/O) and are
 * called at most ~0.5 Hz (throttled by the engine).
 */
fun interface RoadSnapProvider {
    fun snap(latitude: Double, longitude: Double, headingDeg: Float): RoadSnapLatLon?
}
