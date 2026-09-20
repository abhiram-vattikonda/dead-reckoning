package com.sih.idr.navigation

import com.sih.idr.data.GpsSample
import com.sih.idr.data.GnssMeasurement
import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.abs
import kotlin.math.hypot

class RouteAwareFusionEngineTest {

    private fun createInitialState(
        lat: Double = 16.3067,
        lon: Double = 80.4365,
        speed: Float = 0f,
        heading: Float = 0f
    ): NavigationState = NavigationState(
        timestampNanos = 1_000_000_000L,
        latitude = lat,
        longitude = lon,
        altitude = 20.0,
        velEast = 0f,
        velNorth = 0f,
        headingDeg = heading
    )

    @Test
    fun testStandstillZuptArrestsDrift() {
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 0f, heading = 0f))

        var timeNanos = 1_000_000_000L
        // Feed 50 samples (0.5s @ 100 Hz) of stationary IMU data with slight sensor noise (bias = 0.05 m/s^2)
        for (i in 0 until 50) {
            timeNanos += 10_000_000L // 10ms (100 Hz)
            val sample = ImuSample(
                timestampNanos = timeNanos,
                accelX = 0f, accelY = 0.05f, accelZ = 9.81f,
                gyroX = 0.001f, gyroY = 0.001f, gyroZ = 0.001f,
                linearX = 0.02f, linearY = 0.05f, linearZ = 0.01f,
                quatX = 0f, quatY = 0f, quatZ = 0f, quatW = 1f
            )
            engine.processImu(sample)
        }

        assertTrue("ZUPT should engage during standstill", engine.isStationary)
        assertEquals(0.0, engine.forwardSpeed, 1e-6)
        assertEquals(0.0, engine.currentState?.velEast?.toDouble() ?: 0.0, 1e-6)
        assertEquals(0.0, engine.currentState?.velNorth?.toDouble() ?: 0.0, 1e-6)
    }

    @Test
    fun test3DAccelerationRotationWithPhoneTilt() {
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 5f, heading = 0f))

        // Phone tilted 45 degrees pitch:
        // Quaternion for 45 deg pitch around X-axis: sin(pi/8), 0, 0, cos(pi/8)
        val pitchAngle = Math.toRadians(45.0)
        val qx = kotlin.math.sin(pitchAngle / 2.0).toFloat()
        val qw = kotlin.math.cos(pitchAngle / 2.0).toFloat()

        var timeNanos = 1_000_000_000L
        // Accelerating forward in vehicle frame:
        // With a 45 degree tilt, vehicle forward acceleration appears along phone Y and Z!
        val fwdAcc = 2.0 // m/s^2 forward
        val linY = (fwdAcc * kotlin.math.cos(pitchAngle)).toFloat()
        val linZ = (-fwdAcc * kotlin.math.sin(pitchAngle)).toFloat()

        var lastState: NavigationState? = null
        for (i in 0 until 20) {
            timeNanos += 10_000_000L
            val sample = ImuSample(
                timestampNanos = timeNanos,
                accelX = 0f, accelY = linY, accelZ = 9.81f + linZ,
                gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f,
                linearX = 0f, linearY = linY, linearZ = linZ,
                quatX = qx, quatY = 0f, quatZ = 0f, quatW = qw
            )
            lastState = engine.processImu(sample)
        }

        assertNotNull(lastState)
        // Vehicle heading is North (0 deg), forward speed should increase
        assertTrue("Forward speed should increase under forward acceleration", engine.forwardSpeed > 5.0)
        // North velocity should increase, East velocity should remain near zero
        assertTrue("North velocity should be positive", (lastState?.velNorth ?: 0f) > 5.0f)
        assertTrue("East velocity should remain negligible", abs(lastState?.velEast ?: 0f) < 0.2f)
    }

    @Test
    fun testCumulativeRouteArcLengthMonotonicity() {
        val engine = RouteAwareFusionEngine()
        val startLat = 16.3067
        val startLon = 80.4365
        engine.initialize(createInitialState(lat = startLat, lon = startLon, speed = 10f, heading = 90f))

        // Route with highly irregular segment lengths: 5m, 120m, 35m
        val p0 = startLat to startLon
        val p1 = com.sih.idr.utils.GeoUtils.moveEnu(startLat, startLon, 0.0, 5.0)
        val p2 = com.sih.idr.utils.GeoUtils.moveEnu(p1.first, p1.second, 0.0, 120.0)
        val p3 = com.sih.idr.utils.GeoUtils.moveEnu(p2.first, p2.second, 35.0, 0.0)

        engine.setRouteLatLon(listOf(p0, p1, p2, p3))

        // Process a step along East
        var timeNanos = 1_000_000_000L
        for (i in 0 until 50) {
            timeNanos += 10_000_000L
            val sample = ImuSample(
                timestampNanos = timeNanos,
                accelX = 0f, accelY = 0f, accelZ = 9.81f,
                gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f,
                linearX = 0f, linearY = 0f, linearZ = 0f,
                quatX = 0f, quatY = 0f, quatZ = 0.7071f, quatW = 0.7071f // heading ~90 deg East
            )
            engine.processImu(sample)
        }

        assertFalse("Should not trip lateral failure on valid road segments", engine.lateralFailure)
    }

    @Test
    fun testMountOffsetCalibrationFromGps() {
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 0f, heading = 0f))

        // Device yaw is 45 deg, but vehicle is driving along bearing 90 deg (East)
        // GNSS fix moving at 12 m/s along bearing 90 deg
        val gps = GpsSample(
            timestampMillis = 2000L,
            latitude = 16.3068,
            longitude = 80.4367,
            speed = 12.0f,
            bearing = 90.0f,
            accuracy = 3.0f
        )
        engine.onGnssMeasurement(GnssMeasurement(gps))

        assertTrue("Mount calibration should latch from moving GPS fix", engine.mountCalibrated)
        assertEquals(12.0, engine.forwardSpeed, 1e-4)
    }

    @Test
    fun testGnssGlitchRejection() {
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 10f, heading = 0f))

        // Normal GPS fix
        val fix1 = GpsSample(
            timestampMillis = 1000L,
            latitude = 16.3067,
            longitude = 80.4365,
            speed = 10.0f,
            bearing = 0.0f,
            accuracy = 4.0f
        )
        engine.onGnssMeasurement(GnssMeasurement(fix1))
        assertEquals(0, engine.gnssRejects)

        // Glitched teleport: 800m away in 1 second
        val glitch = GpsSample(
            timestampMillis = 2000L,
            latitude = 16.3140, // ~810 meters North
            longitude = 80.4365,
            speed = 10.0f,
            bearing = 0.0f,
            accuracy = 4.0f
        )
        engine.onGnssMeasurement(GnssMeasurement(glitch))

        assertEquals("Glitch should be rejected by plausibility gate", 1, engine.gnssRejects)
    }
}
