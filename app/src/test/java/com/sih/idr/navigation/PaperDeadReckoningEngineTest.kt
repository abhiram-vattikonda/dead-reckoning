package com.sih.idr.navigation

import com.sih.idr.data.GnssMeasurement
import com.sih.idr.data.GpsSample
import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState
import com.sih.idr.utils.GeoUtils
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sin

class PaperDeadReckoningEngineTest {

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
    fun testCoordinateTransformerGravityCalculation() {
        // Identity rotation matrix (flat phone on table, screen facing up)
        val rotM = CoordinateTransformer.rotationMatrixFromQuaternion(0f, 0f, 0f, 1f)
        val gPhone = FloatArray(3)
        CoordinateTransformer.gravityInPhoneFrame(rotM, gPhone)

        // Android accelerometer measures upward normal reaction force (+g) at rest.
        // gPhone[2] must be +9.80665f, so accelZ (9.80665) - gPhone[2] = 0.
        assertEquals(0f, gPhone[0], 1e-4f)
        assertEquals(0f, gPhone[1], 1e-4f)
        assertEquals(9.80665f, gPhone[2], 1e-3f)

        val rawAccelZ = 9.80665f
        val dynamicAz = rawAccelZ - gPhone[2]
        assertEquals("Dynamic vertical acceleration at rest must be 0", 0f, dynamicAz, 1e-3f)
    }

    @Test
    fun testPaperSignalFilter1HzStftAndNoiseElimination() {
        val filter = PaperSignalFilter(motionThreshold = 0.18, lpfCutoffHz = 4.0)

        // 1. Feed low-amplitude sensor noise (bias = 0.05 m/s^2) for 2 seconds (100 samples @ 50 Hz)
        var t = 0.0
        val dt = 0.02
        var lastRes = filter.process(0.05, 0.05, gyroRate = 0.01, dt = dt, currentTimeS = t)
        for (i in 0 until 100) {
            t += dt
            lastRes = filter.process(0.05, 0.05, gyroRate = 0.01, dt = dt, currentTimeS = t)
        }

        // Noise below motion threshold must be eliminated (Section III-A)
        assertEquals("Noise below threshold must be zeroed out", 0.0, lastRes.filteredAx, 1e-6)
        assertEquals("Noise below threshold must be zeroed out", 0.0, lastRes.filteredAy, 1e-6)

        // 2. Feed genuine 1 Hz vehicle acceleration pulse (amplitude 1.5 m/s^2)
        var activeDetected = false
        for (i in 0 until 80) {
            t += dt
            val accel1Hz = 1.5 * sin(2.0 * Math.PI * 1.0 * t)
            val res = filter.process(0.0, accel1Hz, gyroRate = 0.02, dt = dt, currentTimeS = t)
            if (res.isMotionActive) {
                activeDetected = true
            }
        }
        assertTrue("1 Hz acceleration must be detected as active motion", activeDetected)
    }

    @Test
    fun testGpsVelocityDecompositionSectionIIIC() {
        val engine = PaperDeadReckoningEngine()
        val startLat = 16.3067
        val startLon = 80.4365
        engine.initialize(createInitialState(lat = startLat, lon = startLon, speed = 0f, heading = 0f))

        // Phone is oriented pointing North (alpha = 0 deg)
        // Vehicle is travelling East (GPS bearing beta = 90 deg, speed = 10 m/s = 36 km/h)
        val gpsFix = GpsSample(
            timestampMillis = 1000L,
            latitude = startLat,
            longitude = startLon,
            altitude = 20.0,
            speed = 10.0f,
            bearing = 90.0f,
            accuracy = 3.0f
        )
        engine.onGnssMeasurement(GnssMeasurement(gpsFix))

        // Paper Section III-C formulas:
        // beta - alpha = 90 - 0 = 90 deg
        // vgps_x = vgps * sin(90) = 10.0
        // vgps_y = vgps * cos(90) = 0.0
        assertEquals("vx must be 10 m/s along phone lateral axis", 10.0, engine.vx, 0.1)
        assertEquals("vy must be 0 m/s along phone longitudinal axis", 0.0, engine.vy, 0.1)
        assertEquals("Travel heading must be 90 deg (East)", 90f, engine.travelHeadingDeg, 0.5f)
    }

    @Test
    fun testInstantaneousAngleAndHeadingSectionIIID() {
        val engine = PaperDeadReckoningEngine()
        val startLat = 16.3067
        val startLon = 80.4365
        engine.initialize(createInitialState(lat = startLat, lon = startLon, speed = 0f, heading = 0f))

        // GPS fix moving East (beta = 90 deg, speed = 5 m/s) with phone facing North (alpha = 0)
        val gpsFix = GpsSample(
            timestampMillis = 1000L,
            latitude = startLat,
            longitude = startLon,
            speed = 5.0f,
            bearing = 90.0f,
            accuracy = 2.0f
        )
        engine.onGnssMeasurement(GnssMeasurement(gpsFix))

        var timeNanos = 1_000_000_000L
        val dtNanos = 20_000_000L // 50 Hz
        for (i in 0 until 50) { // 1 second of inertial dead reckoning
            timeNanos += dtNanos
            val sample = ImuSample(
                timestampNanos = timeNanos,
                accelX = 0f, accelY = 0f, accelZ = 9.80665f,
                gyroX = 0f, gyroY = 0f, gyroZ = 0f,
                linearX = 0f, linearY = 0f, linearZ = 0f,
                quatX = 0f, quatY = 0f, quatZ = 0f, quatW = 1f
            )
            engine.processImu(sample)
        }

        // Verify that heading remained 90 deg (East) via theta = atan2(dx, dy)
        // dx > 0, dy ~ 0 => theta = 90 deg => phi = 0 + 90 = 90 deg
        assertEquals("Travel heading must track East (90 deg)", 90f, engine.travelHeadingDeg, 1.0f)

        // Verify latitude did not change (moving purely East), and longitude increased
        val finalState = engine.currentState!!
        assertEquals("Latitude should remain constant when moving East", startLat, finalState.latitude, 1e-4)
        assertTrue("Longitude should increase when moving East", finalState.longitude > startLon)
    }

    @Test
    fun testStandstillZuptPreventsCircularDrift() {
        val engine = PaperDeadReckoningEngine()
        val startLat = 16.3067
        val startLon = 80.4365
        engine.initialize(createInitialState(lat = startLat, lon = startLon, speed = 0f, heading = 45f))

        var timeNanos = 1_000_000_000L
        // Feed 150 samples (1.5 seconds) of stationary sensor data with noise
        for (i in 0 until 150) {
            timeNanos += 10_000_000L // 100 Hz
            val sample = ImuSample(
                timestampNanos = timeNanos,
                accelX = 0.04f, accelY = -0.03f, accelZ = 9.80665f,
                gyroX = 0.005f, gyroY = 0.005f, gyroZ = 0.005f,
                linearX = 0.04f, linearY = -0.03f, linearZ = 0f,
                quatX = 0f, quatY = 0f, quatZ = 0f, quatW = 1f
            )
            engine.processImu(sample)
        }

        assertTrue("ZUPT must engage when stationary", engine.isStationary)
        assertEquals("Speed must be 0 at standstill", 0.0, engine.forwardSpeed, 1e-6)

        // Position must NOT have drifted
        val driftMeters = GeoUtils.haversineMeters(startLat, startLon, engine.currentState!!.latitude, engine.currentState!!.longitude)
        assertTrue("Drift at standstill must be < 0.1m, was ${driftMeters}m", driftMeters < 0.1)
    }
}
