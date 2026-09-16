package com.sih.idr.navigation.pdr

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * Absolute heading from gravity + magnetic field (port of the repo's
 * `MagneticFieldOrientation`, re-implemented without EJML using plain 3x3 math).
 *
 * Steps (same as repo):
 *  1. unbiased mag = M − M_bias; map both mag and gravity into the repo's NED-ish
 *     frame via R_NED = [[0,1,0],[1,0,0],[0,0,−1]]
 *  2. roll/pitch from gravity:
 *       r = atan2(Gy, Gz);  p = atan2(−Gx, Gy·sin r + Gz·cos r)
 *     build R_rp (roll/pitch rotation)
 *  3. rotate mag by R_rp (tilt compensation), then
 *       h = −(atan2(−My, Mx) + declination)
 *     (repo hardcoded +11°; declination is configurable here)
 *  4. full orientation R = R_rp · R_h; heading = atan2(R[1][0], R[0][0])
 *
 * Returns NaN when inputs are missing/degenerate; the engine then holds the last
 * magnetic heading (gyro carries the short term — that split is the whole point).
 */
object MagnetometerHeadingProvider {

    fun heading(
        gx: Float, gy: Float, gz: Float,
        mx: Float, my: Float, mz: Float,
        magBiasX: Float = 0f, magBiasY: Float = 0f, magBiasZ: Float = 0f,
        declinationRad: Double = 0.0
    ): Float {
        // 1. bias removal + NED remap (row-vector convention: out = R_NED · v)
        // R_NED swaps x/y and negates z.
        val ux = (mx - magBiasX).toDouble()
        val uy = (my - magBiasY).toDouble()
        val uz = (mz - magBiasZ).toDouble()
        val mX = uy; val mY = ux; val mZ = -uz
        val gX = gy.toDouble(); val gY = gx.toDouble(); val gZ = -gz.toDouble()

        if ((mX == 0.0 && mY == 0.0) || (gX == 0.0 && gY == 0.0 && gZ == 0.0)) return Float.NaN

        // 2. roll / pitch from gravity
        val r = atan2(gY, gZ)
        val p = atan2(-gX, gY * sin(r) + gZ * cos(r))

        // R_rp rows
        val cp = cos(p); val sp = sin(p); val cr = cos(r); val sr = sin(r)
        // 3. M_rp = R_rp · m
        val rpX = cp * mX + sp * sr * mY + sp * cr * mZ
        val rpY = cr * mY - sr * mZ
        // (rpZ unused for heading)

        val h = -(atan2(-rpY, rpX) + declinationRad)

        // 4. R = R_rp · R_h; heading = atan2(R[1][0], R[0][0]).
        // Only the first column of R is needed:
        // R[0][0] = R_rp[0]·R_h[col0], R[1][0] = R_rp[1]·R_h[col0], with R_h col0 = (cos h, sin h, 0).
        val r00 = cp * cos(h) + (sp * sr) * sin(h)
        val r10 = 0.0 * cos(h) + cr * sin(h)
        if (r00 == 0.0 && r10 == 0.0) return Float.NaN
        return atan2(r10, r00).toFloat()
    }

    /** Degrees 0..360 compass-style helper (used only for display, not for math). */
    fun toCompassDeg(mathRad: Float): Float {
        var d = 90.0 - Math.toDegrees(mathRad.toDouble())
        d %= 360.0
        if (d < 0) d += 360.0
        return d.toFloat()
    }
}
