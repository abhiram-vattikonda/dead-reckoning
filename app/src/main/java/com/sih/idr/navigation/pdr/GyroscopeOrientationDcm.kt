package com.sih.idr.navigation.pdr

import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Port of the repo's `GyroscopeEulerOrientation`: orientation as a Direction Cosine
 * Matrix updated by small-angle rotation increments.
 *
 * Consumes the integrator's Δθ (radians, NOT raw rad/s) and applies the repo's
 * EXPLICIT axis remap (their INS frame is NEU-ish, not Android phone frame):
 *   wX = Δθy(sensor), wY = Δθx(sensor), wZ = −Δθz(sensor)
 * Our app documents phone frame as X-right/Y-up/Z-out, so this remap is the
 * documented bridge between the two — do not "fix" it without re-testing heading.
 *
 * Per update:
 *   B = [[0, wz, −wy], [−wz, 0, wx], [wy, −wx, 0]]   (skew-symmetric)
 *   σ = ||w||;  f1 = 1 − σ²/6 + σ⁴/120  (≈ sin σ / σ)
 *              f2 = ½ − σ²/24 + σ⁴/720  (≈ (1−cos σ)/σ²)
 *   A = I + f1·B + f2·B²;   C = C·A
 *   heading = atan2(C[1][0], C[0][0])
 */
class GyroscopeOrientationDcm {

    // Row-major DCM, starts at identity (repo seeds IDENTITY at START).
    private val c = Array(3) { r -> FloatArray(3) { c2 -> if (r == c2) 1f else 0f } }
    private val b = Array(3) { FloatArray(3) }
    private val bSq = Array(3) { FloatArray(3) }
    private val a = Array(3) { FloatArray(3) }
    private val cNew = Array(3) { FloatArray(3) }

    /** Applies one rotation increment; returns gyro-frame heading (radians). */
    fun update(deltaX: Float, deltaY: Float, deltaZ: Float): Float {
        // Repo axis remap (sensor -> INS frame).
        val wX = deltaY
        val wY = deltaX
        val wZ = -deltaZ

        b[0][0] = 0f;  b[0][1] = wZ;  b[0][2] = -wY
        b[1][0] = -wZ; b[1][1] = 0f;  b[1][2] = wX
        b[2][0] = wY;  b[2][1] = -wX; b[2][2] = 0f
        multiply(b, b, bSq)

        val sigma = sqrt(wX * wX + wY * wY + wZ * wZ)
        val f1 = smallAngleF1(sigma)
        val f2 = smallAngleF2(sigma)

        for (r in 0..2) for (col in 0..2) {
            a[r][col] = (if (r == col) 1f else 0f) + f1 * b[r][col] + f2 * bSq[r][col]
        }
        multiply(c, a, cNew)
        for (r in 0..2) for (col in 0..2) c[r][col] = cNew[r][col]

        return atan2(c[1][0], c[0][0])
    }

    /** (sin σ)/σ series: 1 − σ²/3! + σ⁴/5! */
    private fun smallAngleF1(sigma: Float): Float {
        val s2 = sigma.pow(2)
        val s4 = sigma.pow(4)
        return 1f - s2 / 6f + s4 / 120f
    }

    /** (1−cos σ)/σ² series: ½ − σ²/4! + σ⁴/6! */
    private fun smallAngleF2(sigma: Float): Float {
        val s2 = sigma.pow(2)
        val s4 = sigma.pow(4)
        return 0.5f - s2 / 24f + s4 / 720f
    }

    private fun multiply(x: Array<FloatArray>, y: Array<FloatArray>, out: Array<FloatArray>) {
        for (r in 0..2) for (col in 0..2) {
            out[r][col] = x[r][0] * y[0][col] + x[r][1] * y[1][col] + x[r][2] * y[2][col]
        }
    }

    fun reset() {
        for (r in 0..2) for (col in 0..2) c[r][col] = if (r == col) 1f else 0f
    }
}
