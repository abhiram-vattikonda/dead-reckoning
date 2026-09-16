package com.sih.idr.navigation.pdr

/**
 * Intermediate PDR outputs, one per processed IMU sample. Logged to
 * `session_*_pdr.csv` for Python/Jupyter analysis and future ML training.
 * Angles in radians, local frame: x = East, y = North (see GitHubStylePdrEngine docs).
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
    val longitude: Double
)
