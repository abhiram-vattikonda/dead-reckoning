package com.sih.idr.data

/**
 * Generic inertial sample in the PHONE frame (NOT vehicle frame).
 *
 * Phone-frame convention (Android):
 *   X = right across the short edge, Y = up along the long edge, Z = out of the screen.
 * Units: accel m/s^2 (includes gravity), gyro rad/s, magnetometer micro-Tesla.
 * Orientation quaternion rotates vectors from phone frame to world frame (ENU).
 */
data class ImuSample(
    val timestampNanos: Long,
    val accelX: Float,
    val accelY: Float,
    val accelZ: Float,
    val gyroX: Float,
    val gyroY: Float,
    val gyroZ: Float,
    val magnetometerX: Float? = null,
    val magnetometerY: Float? = null,
    val magnetometerZ: Float? = null,
    // Latest rotation-vector quaternion (phone -> world). Null if sensor unavailable yet.
    val quatX: Float? = null,
    val quatY: Float? = null,
    val quatZ: Float? = null,
    val quatW: Float? = null,
    // Android linear acceleration (OS gravity-removed) and gravity vectors.
    // Null when the sensor is absent; the PDR engine falls back to raw-minus-gravity.
    val linearX: Float? = null,
    val linearY: Float? = null,
    val linearZ: Float? = null,
    val gravityX: Float? = null,
    val gravityY: Float? = null,
    val gravityZ: Float? = null
)

/** Generic GNSS fix. Adapter output — no Android types leak past this class. */
data class GpsSample(
    val timestampMillis: Long,
    val latitude: Double,
    val longitude: Double,
    val altitude: Double? = null,
    val speed: Float? = null,      // m/s
    val bearing: Float? = null,    // degrees, direction of travel
    val accuracy: Float? = null,   // meters (1-sigma-ish horizontal)
    val provider: String = "fused"
)

/**
 * Navigation solution produced by a [com.sih.idr.navigation.DeadReckoningEngine].
 * World frame is ENU: velEast/velNorth in m/s. Heading in degrees clockwise from North.
 */
data class NavigationState(
    val timestampNanos: Long,
    val latitude: Double,
    val longitude: Double,
    val altitude: Double = 0.0,
    val velEast: Float = 0f,
    val velNorth: Float = 0f,
    val headingDeg: Float = 0f,
    /** 1-sigma-ish position uncertainty estimate in meters (grows without GNSS). */
    val confidenceMeters: Float = 0f
) {
    val speed: Float get() = kotlin.math.hypot(velEast.toDouble(), velNorth.toDouble()).toFloat()
}

/** One timestamped map point. GPS and DR trajectories are stored SEPARATELY. */
data class TrajectoryPoint(
    val timestampMillis: Long,
    val latitude: Double,
    val longitude: Double
)

/** GNSS measurement delivered to the navigation engine ONLY when the user enables it. */
data class GnssMeasurement(
    val sample: GpsSample
)

/** Aggregate statistics for one test session. All errors/distances in meters. */
data class SessionStats(
    val currentErrorM: Double = Double.NaN,
    val distanceTravelledM: Double = 0.0,
    val maxErrorM: Double = Double.NaN,
    val meanErrorM: Double = Double.NaN,
    val finalErrorM: Double = Double.NaN,
    val driftPct: Double = Double.NaN
)

/**
 * One experiment, kept in memory (no database in v1).
 * gpsTrajectory and drTrajectory are independent lists — never mixed.
 */
data class TestSession(
    val startTimestampMillis: Long,
    var endTimestampMillis: Long? = null,
    /**
     * Offset to convert sensor-boot time → wall clock:
     *   wallMs = (sample.timestampNanos / 1_000_000) + sensorBootOffsetMs
     * Captured once at session start with:
     *   System.currentTimeMillis() - SystemClock.elapsedRealtime()
     * Zero for sessions recorded before this fix.
     */
    val sensorBootOffsetMs: Long = 0L,
    val gpsSamples: MutableList<GpsSample> = mutableListOf(),
    val imuSamples: MutableList<ImuSample> = mutableListOf(),
    val drStates: MutableList<NavigationState> = mutableListOf(),
    val gpsTrajectory: MutableList<TrajectoryPoint> = mutableListOf(),
    val drTrajectory: MutableList<TrajectoryPoint> = mutableListOf(),
    /** Per-sample/step PDR intermediates for ML debugging (see PdrDebug). */
    val pdrDebug: MutableList<com.sih.idr.navigation.pdr.PdrDebug> = mutableListOf(),
    var stats: SessionStats = SessionStats()
)
