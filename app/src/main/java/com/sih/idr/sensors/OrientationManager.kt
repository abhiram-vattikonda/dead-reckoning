package com.sih.idr.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.sih.idr.navigation.CoordinateTransformer
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Owns the rotation-vector sensor and publishes the latest phone->world orientation.
 * Runs on the IMU HandlerThread; snapshots are lock-free via @Volatile fields.
 */
class OrientationManager(context: Context) : SensorEventListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val rotationSensor: Sensor? =
        sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

    val hasRotationVector: Boolean get() = rotationSensor != null

    @Volatile private var qx: Float? = null
    @Volatile private var qy: Float? = null
    @Volatile private var qz: Float? = null
    @Volatile private var qw: Float? = null

    private val _status = MutableStateFlow("Orientation: waiting")
    val status: StateFlow<String> = _status.asStateFlow()

    fun start(): Boolean {
        if (rotationSensor == null) {
            _status.value = "Orientation: NO rotation-vector sensor"
            return false
        }
        // Orientation needs only ~50 Hz; reuse the caller's thread looper via sampling rate.
        _status.value = "Orientation: active"
        return true
    }

    fun register(manager: SensorManager, threadHandler: android.os.Handler): Boolean {
        val s = rotationSensor ?: return false
        return manager.registerListener(this, s, SensorManager.SENSOR_DELAY_GAME, threadHandler)
    }

    fun unregister(manager: SensorManager) = manager.unregisterListener(this)

    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return
        val v = event.values
        // Rotation vector: [x, y, z (, w)] (+ optional heading accuracy at [4]).
        if (v.size >= 4) { qx = v[0]; qy = v[1]; qz = v[2]; qw = v[3] }
        else if (v.size == 3) {
            qx = v[0]; qy = v[1]; qz = v[2]
            val w2 = 1f - (v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
            qw = if (w2 > 0f) kotlin.math.sqrt(w2) else 0f
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    /** Latest quaternion or null if none received yet. */
    fun snapshot(): FloatArray? {
        val x = qx; val y = qy; val z = qz; val w = qw
        return if (x != null && y != null && z != null && w != null) floatArrayOf(x, y, z, w) else null
    }

    fun currentYawDeg(): Float? {
        val q = snapshot() ?: return null
        val m = CoordinateTransformer.rotationMatrixFromQuaternion(q[0], q[1], q[2], q[3])
        return CoordinateTransformer.yawPitchRollDeg(m)[0]
    }
}
