package com.sih.idr.navigation

import kotlin.math.*

/**
 * Accelerometer signal processing and motion filtering directly adapted from:
 * "Dead Reckoning on Smartphones to Reduce GPS Usage" (ICARCV 2014), Section III-A.
 *
 * Flow (Figure 2 in the paper):
 *  1. 4 Hz low-pass filter on raw acceleration to eliminate vibration noise > 4 Hz and avoid aliasing.
 *  2. Downsampling to 20 Hz into 32-sample sliding time slots (1.6 s window).
 *  3. 32-point FFT/DFT spectral analysis focusing on the dominant 1 Hz vehicular motion band.
 *  4. Motion filtering: time slots with less than threshold 1 Hz component are eliminated (zeroed),
 *     preventing integration of sensor bias/noise during constant-speed cruising and stops.
 *  5. Standstill (ZUPT) detection when vehicle is at rest.
 */
class PaperSignalFilter(
    /** Threshold on the 1 Hz FFT component amplitude (m/s^2) below which accel is eliminated as noise. */
    var motionThreshold: Double = 0.18,
    /** Low pass cutoff frequency in Hz (paper specifies 4 Hz). */
    var lpfCutoffHz: Double = 4.0,
    /** Standstill gyro rate threshold in rad/s. */
    var stillGyroThreshold: Double = 0.08
) {
    companion object {
        const val FFT_POINTS = 32
        const val TARGET_SAMPLE_RATE_HZ = 20.0
        const val TARGET_SAMPLE_INTERVAL_S = 1.0 / TARGET_SAMPLE_RATE_HZ // 0.05s

        // Precomputed twiddle factors for k=1 (0.625 Hz ~ 1 Hz band) and k=2 (1.25 Hz)
        private val cosTable1 = DoubleArray(FFT_POINTS) { n -> cos(2.0 * PI * 1.0 * n / FFT_POINTS) }
        private val sinTable1 = DoubleArray(FFT_POINTS) { n -> sin(2.0 * PI * 1.0 * n / FFT_POINTS) }
        private val cosTable2 = DoubleArray(FFT_POINTS) { n -> cos(2.0 * PI * 2.0 * n / FFT_POINTS) }
        private val sinTable2 = DoubleArray(FFT_POINTS) { n -> sin(2.0 * PI * 2.0 * n / FFT_POINTS) }
    }

    // 4 Hz Low-pass filter state
    private var lpfAx = 0.0
    private var lpfAy = 0.0
    private var lpfInitialized = false

    // Downsampling buffer (32 samples @ 20 Hz)
    private val bufferAx = DoubleArray(FFT_POINTS)
    private val bufferAy = DoubleArray(FFT_POINTS)
    private var bufferIndex = 0
    private var bufferCount = 0
    private var lastDownsampleTimeS = -1.0

    // Latest 1 Hz component amplitudes
    var stft1HzMagAx: Double = 0.0
        private set
    var stft1HzMagAy: Double = 0.0
        private set

    // Standstill state
    var isStandstill: Boolean = false
        private set
    private var stillDurationS: Double = 0.0

    data class FilterResult(
        val filteredAx: Double,
        val filteredAy: Double,
        val isMotionActive: Boolean,
        val isStandstill: Boolean,
        val stftEnergy1Hz: Double
    )

    fun reset() {
        lpfAx = 0.0
        lpfAy = 0.0
        lpfInitialized = false
        bufferAx.fill(0.0)
        bufferAy.fill(0.0)
        bufferIndex = 0
        bufferCount = 0
        lastDownsampleTimeS = -1.0
        stft1HzMagAx = 0.0
        stft1HzMagAy = 0.0
        isStandstill = false
        stillDurationS = 0.0
    }

    /**
     * Process one dynamic acceleration sample (gravity removed) on phone's X and Y axes.
     * @param rawAx Dynamic acceleration along phone X axis (m/s^2)
     * @param rawAy Dynamic acceleration along phone Y axis (m/s^2)
     * @param gyroRate Total angular velocity magnitude (rad/s)
     * @param dt Time interval from previous sample in seconds
     * @param currentTimeS Elapsed time in seconds
     */
    fun process(
        rawAx: Double,
        rawAy: Double,
        gyroRate: Double,
        dt: Double,
        currentTimeS: Double
    ): FilterResult {
        // Step 1: 4 Hz Low-Pass Filter
        if (!lpfInitialized) {
            lpfAx = rawAx
            lpfAy = rawAy
            lpfInitialized = true
        } else {
            val alpha = (2.0 * PI * lpfCutoffHz * dt) / (1.0 + 2.0 * PI * lpfCutoffHz * dt).coerceIn(0.01, 0.95)
            lpfAx += alpha * (rawAx - lpfAx)
            lpfAy += alpha * (rawAy - lpfAy)
        }

        // Step 2: Downsampling to 20 Hz into 32-sample sliding window
        if (lastDownsampleTimeS < 0 || currentTimeS - lastDownsampleTimeS >= TARGET_SAMPLE_INTERVAL_S) {
            lastDownsampleTimeS = currentTimeS
            bufferAx[bufferIndex] = lpfAx
            bufferAy[bufferIndex] = lpfAy
            bufferIndex = (bufferIndex + 1) % FFT_POINTS
            if (bufferCount < FFT_POINTS) bufferCount++

            // Step 3: Compute STFT 1 Hz component when buffer has sufficient samples (>= 16)
            if (bufferCount >= 16) {
                stft1HzMagAx = compute1HzMagnitude(bufferAx, bufferIndex, bufferCount)
                stft1HzMagAy = compute1HzMagnitude(bufferAy, bufferIndex, bufferCount)
            }
        }

        val total1HzEnergy = hypot(stft1HzMagAx, stft1HzMagAy)

        // Step 4: Standstill (ZUPT) detection
        val accelMag = hypot(lpfAx, lpfAy)
        val isQuiet = gyroRate < stillGyroThreshold && accelMag < 0.35 && total1HzEnergy < motionThreshold * 0.75
        if (isQuiet) {
            stillDurationS += dt
        } else {
            stillDurationS = 0.0
        }
        isStandstill = stillDurationS >= 0.8

        // Step 5: Paper Section III-A Motion Filtering:
        // "time slots with less than a threshold value of FFT 1st component value have been eliminated in the filtering process."
        val isMotionActive = !isStandstill && total1HzEnergy >= motionThreshold

        val finalAx: Double
        val finalAy: Double

        if (isStandstill) {
            finalAx = 0.0
            finalAy = 0.0
        } else if (!isMotionActive) {
            // Below motion threshold: eliminate acceleration noise
            finalAx = 0.0
            finalAy = 0.0
        } else {
            // Active acceleration or braking: pass filtered acceleration
            finalAx = lpfAx
            finalAy = lpfAy
        }

        return FilterResult(
            filteredAx = finalAx,
            filteredAy = finalAy,
            isMotionActive = isMotionActive,
            isStandstill = isStandstill,
            stftEnergy1Hz = total1HzEnergy
        )
    }

    /**
     * Discrete Fourier Transform magnitude for the 1 Hz frequency band.
     * Evaluates DFT bins 1 and 2 (0.625 Hz and 1.25 Hz) on the circular buffer.
     */
    private fun compute1HzMagnitude(buffer: DoubleArray, head: Int, count: Int): Double {
        if (count < 8) return 0.0
        var real1 = 0.0
        var imag1 = 0.0
        var real2 = 0.0
        var imag2 = 0.0

        for (i in 0 until count) {
            val idx = (head - count + i + FFT_POINTS) % FFT_POINTS
            val sample = buffer[idx]
            real1 += sample * cosTable1[i]
            imag1 -= sample * sinTable1[i]
            real2 += sample * cosTable2[i]
            imag2 -= sample * sinTable2[i]
        }

        val mag1 = hypot(real1, imag1) / count
        val mag2 = hypot(real2, imag2) / count
        // Interpolated 1 Hz component between bin 1 (0.625 Hz) and bin 2 (1.25 Hz)
        return max(mag1, mag2)
    }
}
