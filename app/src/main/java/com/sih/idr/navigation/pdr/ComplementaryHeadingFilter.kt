package com.sih.idr.navigation.pdr

/**
 * Port of the repo's `ExtraFunctions.calcCompHeading`:
 *   heading = MAG_WEIGHT · magHeading + GYRO_WEIGHT · gyroHeading
 * with defaults 0.02 / 0.98 — gyro carries the smooth short term, magnetometer
 * slowly anchors the absolute reference.
 *
 * Angle wrap is handled the way the repo does it: negative inputs are reduced
 * with `% 2π` (Kotlin/Java `%` keeps the sign, so −0.5 stays −0.5 — quirky but
 * faithful), and results above +π wrap back by `% π − π`. Do not "simplify" this
 * to a naive average: near the ±π discontinuity a plain mean points backwards.
 */
class ComplementaryHeadingFilter(
    var magWeight: Float = 0.02f,
    var gyroWeight: Float = 0.98f
) {
    fun fuse(magHeading: Float, gyroHeading: Float): Float {
        var m = magHeading.toDouble()
        var g = gyroHeading.toDouble()
        val tau = 2.0 * Math.PI
        if (m < 0) m %= tau
        if (g < 0) g %= tau
        var comp = magWeight * m + gyroWeight * g
        if (comp > Math.PI) comp = (comp % Math.PI) + -Math.PI
        return comp.toFloat()
    }
}
