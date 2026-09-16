package com.sih.idr.navigation.pdr

/**
 * Port of the repo's `GyroscopeDeltaOrientation`:
 *   unbiased = raw − bias;  |unbiased| ≤ threshold → 0 (high-pass);
 *   Δθ = unbiased · dt, with dt from SENSOR timestamps (never assumed fixed).
 *
 * The returned array is reused across calls — consume immediately, do not retain.
 */
class GyroscopeIntegrator(
    var thresholdRps: Float = 0.0025f,
    biasX: Float = 0f,
    biasY: Float = 0f,
    biasZ: Float = 0f
) {
    private var firstRun = true
    private var lastTimeSec = 0.0
    private val bias = floatArrayOf(biasX, biasY, biasZ)
    private val delta = FloatArray(3)

    fun setBias(bx: Float, by: Float, bz: Float) {
        bias[0] = bx; bias[1] = by; bias[2] = bz
    }

    /**
     * Integrates one raw gyro sample (rad/s, phone frame). First call only latches
     * the timestamp and returns zeros (no dt available yet — same as the repo).
     */
    fun update(timestampNanos: Long, gx: Float, gy: Float, gz: Float): FloatArray {
        val tSec = timestampNanos / 1e9
        if (firstRun) {
            firstRun = false
            lastTimeSec = tSec
            delta[0] = 0f; delta[1] = 0f; delta[2] = 0f
            return delta
        }
        var ux = gx - bias[0]
        var uy = gy - bias[1]
        var uz = gz - bias[2]
        // Quick high-pass: sub-threshold rates are treated as zero (kept because the
        // repo uses it to suppress gyro noise at rest; remove only with a re-test).
        if (kotlin.math.abs(ux) <= thresholdRps) ux = 0f
        if (kotlin.math.abs(uy) <= thresholdRps) uy = 0f
        if (kotlin.math.abs(uz) <= thresholdRps) uz = 0f
        var dt = tSec - lastTimeSec
        if (dt <= 0.0 || dt > 0.2) dt = 0.01 // timestamp glitch/gap guard
        delta[0] = (ux * dt).toFloat()
        delta[1] = (uy * dt).toFloat()
        delta[2] = (uz * dt).toFloat()
        lastTimeSec = tSec
        return delta
    }

    fun reset() {
        firstRun = true
        lastTimeSec = 0.0
        delta[0] = 0f; delta[1] = 0f; delta[2] = 0f
    }
}
