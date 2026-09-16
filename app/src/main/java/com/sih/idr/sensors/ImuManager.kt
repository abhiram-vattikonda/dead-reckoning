package com.sih.idr.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import com.sih.idr.data.ImuSample
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Android SensorManager -> [ImuSample] adapter.
 *
 * The navigation engine never touches SensorManager; it only sees ImuSample, so a
 * future ExternalImuAdapter (e.g. 200 Hz FOG-IMU) can feed the same engine.
 *
 * Threading: all callbacks run on a dedicated "imu-thread" HandlerThread, never the
 * UI thread. Samples are emitted on BOTH accel and gyro events (whichever arrives),
 * carrying a zero-order hold of the other sensor + latest mag/orientation. Each
 * sample keeps its trigger event's hardware timestamp (SENSOR_DELAY_GAME ≈ 50–200 Hz
 * depending on device; switch to SENSOR_DELAY_FASTEST in [start] if you need max rate).
 */
class ImuManager(context: Context, val orientation: OrientationManager) : SensorEventListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accel: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyro: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val mag: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    // PDR needs OS gravity-removed linear accel + a gravity vector (repo used
    // TYPE_LINEAR_ACCELERATION and TYPE_GRAVITY). Both are zero-order-held like mag.
    private val linAccel: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
    private val gravity: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)

    private var thread: HandlerThread? = null
    private var handler: Handler? = null

    @Volatile var listener: ((ImuSample) -> Unit)? = null

    // Latest values (written on imu-thread, read on imu-thread — volatile for safety).
    @Volatile private var ax = 0f; @Volatile private var ay = 0f; @Volatile private var az = 0f
    @Volatile private var hasAccel = false
    @Volatile private var gx = 0f; @Volatile private var gy = 0f; @Volatile private var gz = 0f
    @Volatile private var hasGyro = false
    @Volatile private var mx: Float? = null; @Volatile private var my: Float? = null; @Volatile private var mz: Float? = null
    @Volatile private var lx: Float? = null; @Volatile private var ly: Float? = null; @Volatile private var lz: Float? = null
    @Volatile private var gravX: Float? = null; @Volatile private var gravY: Float? = null; @Volatile private var gravZ: Float? = null

    @Volatile var sampleCount: Long = 0L
        private set

    /**
     * Measured callback rate (emitted samples/sec, rolling 1 s window).
     * Proves the sensor pipeline is alive independent of GPS/location state.
     */
    @Volatile var measuredHz: Float = 0f
        private set
    private var rateWindowCount = 0
    private var rateWindowStartNs = System.nanoTime()
    private var lastRateStatusMs = 0L

    private val _status = MutableStateFlow("IMU: idle")
    val status: StateFlow<String> = _status.asStateFlow()

    val hasAccelerometer: Boolean get() = accel != null
    val hasGyroscope: Boolean get() = gyro != null
    val hasMagnetometer: Boolean get() = mag != null
    val hasLinearAccel: Boolean get() = linAccel != null
    val hasGravity: Boolean get() = gravity != null

    fun start() {
        if (thread != null) return
        val t = HandlerThread("imu-thread").also { it.start() }
        thread = t
        val h = Handler(t.looper)
        handler = h
        var parts = mutableListOf<String>()
        if (accel != null) {
            sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_GAME, h)
            parts += "accel"
        }
        if (gyro != null) {
            sensorManager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_GAME, h)
            parts += "gyro"
        }
        if (mag != null) {
            sensorManager.registerListener(this, mag, SensorManager.SENSOR_DELAY_GAME, h)
            parts += "mag"
        }
        if (linAccel != null) {
            sensorManager.registerListener(this, linAccel, SensorManager.SENSOR_DELAY_GAME, h)
            parts += "linAcc"
        }
        if (gravity != null) {
            sensorManager.registerListener(this, gravity, SensorManager.SENSOR_DELAY_GAME, h)
            parts += "grav"
        }
        orientation.register(sensorManager, h)
        if (orientation.hasRotationVector) parts += "rotVec"
        sampleCount = 0
        _status.value = if (parts.isEmpty()) "IMU: NO SENSORS" else "IMU: " + parts.joinToString("+")
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        orientation.unregister(sensorManager)
        thread?.quitSafely()
        thread = null
        handler = null
        _status.value = "IMU: idle"
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                ax = event.values[0]; ay = event.values[1]; az = event.values[2]
                hasAccel = true
                if (hasGyro) emit(event.timestamp)
            }
            Sensor.TYPE_GYROSCOPE -> {
                gx = event.values[0]; gy = event.values[1]; gz = event.values[2]
                hasGyro = true
                if (hasAccel) emit(event.timestamp)
            }
            Sensor.TYPE_MAGNETIC_FIELD -> {
                mx = event.values[0]; my = event.values[1]; mz = event.values[2]
            }
            Sensor.TYPE_LINEAR_ACCELERATION -> {
                lx = event.values[0]; ly = event.values[1]; lz = event.values[2]
            }
            Sensor.TYPE_GRAVITY -> {
                gravX = event.values[0]; gravY = event.values[1]; gravZ = event.values[2]
            }
        }
    }

    private fun emit(timestampNanos: Long) {
        val q = orientation.snapshot()
        listener?.invoke(
            ImuSample(
                timestampNanos = timestampNanos,
                accelX = ax, accelY = ay, accelZ = az,
                gyroX = gx, gyroY = gy, gyroZ = gz,
                magnetometerX = mx, magnetometerY = my, magnetometerZ = mz,
                quatX = q?.get(0), quatY = q?.get(1), quatZ = q?.get(2), quatW = q?.get(3),
                linearX = lx, linearY = ly, linearZ = lz,
                gravityX = gravX, gravityY = gravY, gravityZ = gravZ
            )
        )
        sampleCount++
        // Rolling 1 s rate window; status text refreshed at most 1/s (StateFlow is thread-safe).
        rateWindowCount++
        val nowNs = System.nanoTime()
        val windowNs = nowNs - rateWindowStartNs
        if (windowNs >= 1_000_000_000L) {
            measuredHz = rateWindowCount * 1e9f / windowNs
            rateWindowCount = 0
            rateWindowStartNs = nowNs
            val nowMs = System.currentTimeMillis()
            if (nowMs - lastRateStatusMs >= 1000L) {
                lastRateStatusMs = nowMs
                val base = _status.value.substringBefore(" @")
                _status.value = "$base @ ${measuredHz.toInt()}Hz"
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}
