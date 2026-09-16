package com.sih.idr.navigation.pdr

/**
 * Stride-length source, kept behind an interface so a learned model can replace
 * the fixed default later WITHOUT touching the engine or UI:
 *
 *   class LearnedStrideModel(...) : StrideLengthProvider { ... }
 *
 * then pass it into [com.sih.idr.navigation.GitHubStylePdrEngine].
 */
interface StrideLengthProvider {
    /** Meters advanced per detected step. */
    fun strideLengthMeters(): Float
}

/** Fixed calibration value (the pedestrian baseline). */
class FixedStrideLength(private val meters: Float) : StrideLengthProvider {
    override fun strideLengthMeters(): Float = meters
}
