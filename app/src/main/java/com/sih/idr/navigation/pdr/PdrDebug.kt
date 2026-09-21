package com.sih.idr.navigation.pdr

/**
 * Intermediate engine outputs, one per processed IMU sample. Logged to
 * `session_*_pdr.csv` for Python/Jupyter analysis and future ML training.
 * Angles in radians, local frame: x = East, y = North (see GitHubStylePdrEngine docs).
 *
 * Vehicle-fusion fields (forwardSpeedMps … snapMode) default to NaN/"" so the
 * pedestrian baseline keeps compiling unchanged; the route-aware fusion engine
 * populates them on EVERY sample so nav/pdr/imu rows stay joinable on
 * timestamp_nanos with no silent NaN gaps in the vehicle columns.
 */
data class PdrDebug(
    val timestampNanos: Long,
    val accMagnitude: Float,
    val stepDetected: Boolean,
    val strideLength: Float,
    val gyroHeadingRad: Float,
    val magHeadingRad: Float,
    val fusedHeadingRad: Float,
    val xEast: Float,
    val yNorth: Float,
    val latitude: Double,
    val longitude: Double,
    // --- vehicle fusion extension (consistent logging, see CsvExporter) ---
    /** Longitudinal speed along travel heading (m/s). */
    val forwardSpeedMps: Float = Float.NaN,
    /** Filtered yaw rate, deg/s, +clockwise (turn detector input). */
    val yawRateDegS: Float = Float.NaN,
    /** Residual lateral (right-positive) velocity after NHC damping (m/s). */
    val lateralVelMps: Float = Float.NaN,
    /** Which constraint produced this row: ROUTE/TRAIL/AUTO/FREE/PDR/ZUPT. */
    val snapMode: String = ""
)
