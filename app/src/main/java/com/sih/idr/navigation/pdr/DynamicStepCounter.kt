package com.sih.idr.navigation.pdr

/**
 * Faithful Kotlin port of the repo's `DynamicStepCounter` (continuous-threshold mode,
 * which is the path the repo actually uses).
 *
 * Per linear-acceleration-magnitude sample `acc`:
 * - moving average: avg = avg·((n−1)/n) + acc/n   (n = 1-based sample count)
 * - upper = avg + sensitivity, lower = avg − sensitivity
 * - step iff acc > upper AND no peak latched; latch clears only when acc < lower
 *   (hysteresis — prevents multi-triggering on one wide peak).
 *
 * NOTE on input: the repo fed this detector `|ax+ay+az|` (it passed the *sum* into
 * its norm function — almost certainly a bug). We deliberately feed the proper
 * magnitude sqrt(ax²+ay²+az²); the detector math itself is unchanged.
 */
class DynamicStepCounter(sensitivity: Double = 1.0) {

    var sensitivity: Double = sensitivity
        private set

    var stepCount: Int = 0
        private set

    var upperThreshold: Double = 10.8
        private set
    var lowerThreshold: Double = 8.8
        private set
    var avgAcc: Double = 0.0
        private set

    private var runCount = 0
    private var firstRun = true
    private var peakFound = false

    fun setSensitivity(s: Double) {
        sensitivity = s
    }

    /** Returns true exactly once per detected step. */
    fun findStep(acc: Double): Boolean {
        updateThresholds(acc)
        if (acc > upperThreshold) {
            if (!peakFound) {
                stepCount++
                peakFound = true
                return true
            }
        } else if (acc < lowerThreshold) {
            if (peakFound) peakFound = false
        }
        return false
    }

    private fun updateThresholds(acc: Double) {
        runCount++
        if (firstRun) {
            upperThreshold = acc + sensitivity
            lowerThreshold = acc - sensitivity
            avgAcc = acc
            firstRun = false
            return
        }
        val n = runCount.toDouble()
        avgAcc = avgAcc * ((n - 1.0) / n) + acc / n
        upperThreshold = avgAcc + sensitivity
        lowerThreshold = avgAcc - sensitivity
    }

    fun reset() {
        stepCount = 0
        upperThreshold = 10.8
        lowerThreshold = 8.8
        avgAcc = 0.0
        runCount = 0
        firstRun = true
        peakFound = false
    }
}
