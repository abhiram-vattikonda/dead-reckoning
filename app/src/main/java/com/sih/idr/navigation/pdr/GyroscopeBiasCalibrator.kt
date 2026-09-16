package com.sih.idr.navigation.pdr

/**
 * Standalone gyro-bias calibration (port of the repo's `GyroscopeBias`).
 * Moving average over [trials] samples — the phone must be STATIONARY while this runs:
 *   bias = bias·((n−1)/n) + raw·(1/n)
 * Deliberately separate from the DR engine (spec: keep calibration code separate).
 */
class GyroscopeBiasCalibrator(val trials: Int = 200) {

    private var runCount = 0
    private val bias = FloatArray(3)

    val isCalibrated: Boolean get() = runCount >= trials

    /** Feeds one raw gyro sample; returns true once calibration is complete. */
    fun addSample(gx: Float, gy: Float, gz: Float): Boolean {
        runCount++
        if (runCount >= trials) return true
        if (runCount == 1) {
            bias[0] = gx; bias[1] = gy; bias[2] = gz
            return false
        }
        val n = runCount.toFloat()
        bias[0] = bias[0] * ((n - 1f) / n) + gx * (1f / n)
        bias[1] = bias[1] * ((n - 1f) / n) + gy * (1f / n)
        bias[2] = bias[2] * ((n - 1f) / n) + gz * (1f / n)
        return false
    }

    fun biasX(): Float = bias[0]
    fun biasY(): Float = bias[1]
    fun biasZ(): Float = bias[2]

    fun setManualBias(bx: Float, by: Float, bz: Float) {
        bias[0] = bx; bias[1] = by; bias[2] = bz
        runCount = trials // mark complete so auto-calibration is skipped
    }

    fun reset() {
        runCount = 0
        bias[0] = 0f; bias[1] = 0f; bias[2] = 0f
    }
}
