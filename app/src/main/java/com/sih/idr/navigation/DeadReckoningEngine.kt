package com.sih.idr.navigation

import com.sih.idr.data.GnssMeasurement
import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState

/**
 * Replaceable navigation-engine contract.
 *
 * The UI / ViewModel only ever talks to this interface — never to a concrete engine.
 * To add the future AI model, create `MMMDeadReckoningEngine : DeadReckoningEngine`
 * (IMU -> preprocessing -> model inference -> NavigationState) and swap the single
 * construction site in the ViewModel. Nothing else changes.
 *
 * HARD BOUNDARY: after [initialize], the engine must run on IMU alone. GNSS may only
 * enter through [onGnssMeasurement], which the app calls ONLY when the user enables
 * "GNSS Available To Navigation Engine". The baseline implementation ignores GNSS
 * entirely (default no-op), keeping ground-truth GPS fully isolated.
 */
interface DeadReckoningEngine {
    /** Seed absolute position / velocity / heading. Called once per START press. */
    fun initialize(initialState: NavigationState)

    /** Consume one IMU sample, return the updated navigation solution. */
    fun processImu(sample: ImuSample): NavigationState

    /**
     * Optional GNSS input for future fusion engines. Default: ignore.
     * The app layer guarantees this is never called while the GNSS switch is OFF.
     */
    fun onGnssMeasurement(measurement: GnssMeasurement) {}

    fun reset()

    val currentState: NavigationState?
}
