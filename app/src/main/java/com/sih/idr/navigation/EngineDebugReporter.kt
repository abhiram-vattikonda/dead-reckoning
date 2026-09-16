package com.sih.idr.navigation

import com.sih.idr.navigation.pdr.PdrDebug

/**
 * Engines expose their latest intermediates through this instead of a concrete
 * cast, so logging/CSV keeps working no matter which engine is active
 * (PDR step engine, route-aware fusion engine, future MMM engine).
 */
interface EngineDebugReporter {
    val lastDebug: PdrDebug?
}
