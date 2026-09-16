package com.sih.idr.navigation.pdr

/**
 * All tunables for [com.sih.idr.navigation.GitHubStylePdrEngine] in one place.
 * Defaults reproduce nisargnp/DeadReckoning behavior, except:
 * - [declinationDeg]: repo hardcoded 11° (its locale); default 0 here — set yours.
 * - stride default 0.75 m (repo default 2.5 was feet-ish for its calibration UI).
 */
data class PdrConfig(
    /** Meters advanced per detected step. Replace via [StrideLengthProvider]. */
    val strideLengthMeters: Float = 0.75f,
    /** Step-detector hysteresis half-width (repo default 1.0). */
    val stepSensitivity: Double = 1.0,
    /** Gyro high-pass: |ω| below this (rad/s) is treated as zero (repo 0.0025). */
    val gyroThresholdRps: Float = 0.0025f,
    /** Samples averaged for auto gyro-bias calibration at engine start. */
    val gyroBiasTrials: Int = 200,
    /** When true, first [gyroBiasTrials] gyro samples (assumed stationary) set the bias. */
    val autoCalibrateGyroBias: Boolean = true,
    val gyroBiasX: Float = 0f,
    val gyroBiasY: Float = 0f,
    val gyroBiasZ: Float = 0f,
    val magBiasX: Float = 0f,
    val magBiasY: Float = 0f,
    val magBiasZ: Float = 0f,
    /** Complementary-filter weights (repo 0.02 / 0.98). */
    val magWeight: Float = 0.02f,
    val gyroWeight: Float = 0.98f,
    /** Local magnetic declination, degrees East-positive. Added inside mag heading. */
    val declinationDeg: Double = 0.0
)
