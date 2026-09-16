package com.sih.idr.sensors

import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import android.os.Looper
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import com.sih.idr.data.GpsSample
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Android Fused Location Provider -> [GpsSample] adapter.
 *
 * This manager is INDEPENDENT of the navigation engine. It always reports the true
 * GPS fix to the UI/recorder. Whether the engine may *consume* GNSS is decided at
 * the app layer ("GNSS Available To Navigation Engine" switch) — see ViewModel.
 */
class GpsManager(context: Context) {

    private val client = LocationServices.getFusedLocationProviderClient(context.applicationContext)

    private val _lastFix = MutableStateFlow<GpsSample?>(null)
    val lastFix: StateFlow<GpsSample?> = _lastFix.asStateFlow()

    private val _status = MutableStateFlow("GPS: idle")
    val status: StateFlow<String> = _status.asStateFlow()

    /** Raw per-fix callback (recording path). Called on the main looper. */
    @Volatile var listener: ((GpsSample) -> Unit)? = null

    private var running = false

    private val callback = object : LocationCallback() {
        override fun onLocationResult(result: LocationResult) {
            for (loc in result.locations) {
                val s = loc.toSample()
                _lastFix.value = s
                listener?.invoke(s)
            }
            _status.value = "GPS: active (${result.locations.size} fix)"
        }
    }

    @SuppressLint("MissingPermission") // Caller guarantees location permission.
    fun start() {
        if (running) return
        running = true
        _status.value = "GPS: waiting for fix…"
        val request = LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, 1000L)
            .setMinUpdateDistanceMeters(0f)
            .setMaxUpdateDelayMillis(2000L)
            .build()
        client.requestLocationUpdates(request, callback, Looper.getMainLooper())
        // Warm up with the last known fix immediately.
        client.lastLocation.addOnSuccessListener { loc: Location? ->
            if (loc != null) _lastFix.value = loc.toSample()
        }
    }

    fun stop() {
        if (!running) return
        running = false
        client.removeLocationUpdates(callback)
        _status.value = "GPS: idle"
    }

    fun isRunning(): Boolean = running

    private fun Location.toSample() = GpsSample(
        timestampMillis = time,
        latitude = latitude,
        longitude = longitude,
        altitude = if (hasAltitude()) altitude else null,
        speed = if (hasSpeed()) speed else null,
        bearing = if (hasBearing()) bearing else null,
        accuracy = if (hasAccuracy()) accuracy else null,
        provider = provider ?: "fused"
    )
}
