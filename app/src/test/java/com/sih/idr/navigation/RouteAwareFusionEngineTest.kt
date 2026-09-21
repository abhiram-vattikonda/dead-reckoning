package com.sih.idr.navigation

import com.sih.idr.data.GpsSample
import com.sih.idr.data.GnssMeasurement
import com.sih.idr.data.ImuSample
import com.sih.idr.data.NavigationState
import com.sih.idr.utils.GeoUtils
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
    ): NavigationState {
        // Honor the requested speed: the engine seeds forwardSpeed from the
        // state's velocity norm, so a "moving" start needs non-zero velocity.
        val hRad = Math.toRadians(heading.toDouble())
        return NavigationState(
            timestampNanos = 1_000_000_000L,
            latitude = lat,
            longitude = lon,
            altitude = 20.0,
            velEast = (speed * kotlin.math.sin(hRad)).toFloat(),
            velNorth = (speed * kotlin.math.cos(hRad)).toFloat(),
            headingDeg = heading
        )
    }

    /**
     * Seeds motion: [initialize] always starts at zero velocity (it derives
     * forwardSpeed from the state's speed norm), so tests that need a moving
     * vehicle calibrate via a GNSS fix at the origin with speed + bearing.
     */
    private fun seedMoving(engine: RouteAwareFusionEngine, bearingDeg: Float, speedMps: Float) {
        engine.onGnssMeasurement(
            GnssMeasurement(
                GpsSample(
                    timestampMillis = 1000L,
                    latitude = 16.3067,
                    longitude = 80.4365,
                    speed = speedMps,
                    bearing = bearingDeg,
                    accuracy = 3.0f
                )
            )
        )
    }

    /** Quaternion (x,y,z,w) producing device yaw [yawDeg] (clockwise from North). */
    private fun yawQuat(yawDeg: Double): FloatArray {
        // Device yaw θ ⟺ Rz(ψ=-θ); quat = (0, 0, sin(ψ/2), cos(ψ/2)).
        val half = Math.toRadians(-yawDeg) / 2.0
        return floatArrayOf(0f, 0f, kotlin.math.sin(half).toFloat(), kotlin.math.cos(half).toFloat())
    }

    /** World-frame (east, north) accel → phone-frame triple for a yaw-only attitude. */
    private fun phoneAccelForWorld(worldE: Double, worldN: Double, yawDeg: Double): Triple<Float, Float, Float> {
        val q = yawQuat(yawDeg)
        val r = CoordinateTransformer.rotationMatrixFromQuaternion(q[0], q[1], q[2], q[3])
        // world = R * phone  →  phone = Rᵀ * world (R orthonormal).
        val px = (r[0] * worldE + r[3] * worldN).toFloat()
        val py = (r[1] * worldE + r[4] * worldN).toFloat()
        val pz = (r[2] * worldE + r[5] * worldN).toFloat()
        return Triple(px, py, pz)
    }

    private fun imu(
        timeNanos: Long,
        yawDeg: Double,
        worldE: Double = 0.0,
        worldN: Double = 0.0,
        gyroZ: Float = 0f
    ): ImuSample {
        val q = yawQuat(yawDeg)
        val (px, py, pz) = phoneAccelForWorld(worldE, worldN, yawDeg)
        return ImuSample(
            timestampNanos = timeNanos,
            accelX = px, accelY = py, accelZ = 9.81f + pz,
            gyroX = 0f, gyroY = 0f, gyroZ = gyroZ,
            linearX = px, linearY = py, linearZ = pz,
            quatX = q[0], quatY = q[1], quatZ = q[2], quatW = q[3]
        )
    }

    @Test
    fun testStandstillZuptArrestsDrift() {
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 0f, heading = 0f))

        var timeNanos = 1_000_000_000L
        // Feed 140 samples (1.4s @ 100 Hz) of stationary IMU data with slight sensor noise (bias = 0.05 m/s^2)
        // STANDSTILL_MIN_TIME_S is 1.2s to prevent cruising false-triggers
        for (i in 0 until 140) {
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
    fun testNoEndpointLockWhenPassingRouteSegment() {
        val engine = RouteAwareFusionEngine()
        val startLat = 16.3067
        val startLon = 80.4365
        engine.initialize(createInitialState(lat = startLat, lon = startLon, speed = 5f, heading = 90f))

        // Short segment of 10 meters East
        val p0 = startLat to startLon
        val p1 = GeoUtils.moveEnu(startLat, startLon, 0.0, 10.0)
        engine.setRouteLatLon(listOf(p0, p1))

        // Move 25 meters East (15 meters beyond the segment end)
        val beyond = GeoUtils.moveEnu(startLat, startLon, 0.0, 25.0)
        val enu25 = GeoUtils.geodeticToEnu(beyond.first, beyond.second, 0.0, startLat, startLon, 0.0)

        // Segment 0: from (0,0) to (10,0)
        val snapped = engine.constrainToRoute(enu25.x, enu25.y)
        // Since tRaw = 25/10 = 2.5 > 1.05, it must NOT snap to the end vertex (10.0, 0.0)!
        assertNull("Vehicle beyond segment must not be locked to endpoint", snapped)
    }

    @Test
    fun testPedestrianStepImpulsesAdvanceSpeed() {
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 0f, heading = 0f))

        // Simulate walking footsteps: periodic vertical acceleration peak (~14 m/s^2)
        // every 500ms, each followed by a stance valley (~7.5 m/s^2) so the
        // hysteresis detector unlatches between steps (as a real gait does).
        var timeNanos = 1_000_000_000L
        for (step in 0 until 5) {
            for (i in 0 until 50) { // 500ms per cycle
                timeNanos += 10_000_000L
                val az = when {
                    i == 10 -> 14.5f // heel-strike peak
                    i in 25..30 -> 7.5f // stance valley (unlatches the detector)
                    else -> 9.81f
                }
                val sample = ImuSample(
                    timestampNanos = timeNanos,
                    accelX = 0f, accelY = 0.5f, accelZ = az,
                    gyroX = 0.01f, gyroY = 0.01f, gyroZ = 0.01f,
                    linearX = 0f, linearY = 0.5f, linearZ = az - 9.81f,
                    quatX = 0f, quatY = 0f, quatZ = 0f, quatW = 1f
                )
                engine.processImu(sample)
            }
        }

        assertTrue("Step counter should register footsteps", engine.stepCount >= 3)
        assertTrue("Pedestrian speed should be maintained", engine.forwardSpeed > 0.5)
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
        val p1 = GeoUtils.moveEnu(startLat, startLon, 0.0, 5.0)
        val p2 = GeoUtils.moveEnu(p1.first, p1.second, 0.0, 120.0)
        val p3 = GeoUtils.moveEnu(p2.first, p2.second, 35.0, 0.0)

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

    // ------------------------------------------------- new behavior: heading fusion
    @Test
    fun testDominantVelocityVectorTracksDirectionOfTravel() {
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 8f, heading = 90f))
        seedMoving(engine, 90f, 8f)

        // Cruise east at 8 m/s, zero accel, device yaw pinned east.
        var timeNanos = 1_000_000_000L
        repeat(150) {
            timeNanos += 10_000_000L
            engine.processImu(imu(timeNanos, yawDeg = 90.0))
        }

        assertTrue("Cruise must not trigger ZUPT", !engine.isStationary)
        assertTrue("Forward speed must survive the cruise", engine.forwardSpeed > 5.0)
        val domErr = abs(GeoUtils.angleDiffDeg(engine.dominantHeadingDeg.toDouble(), 90.0))
        assertTrue("Dominant vector must point east, err=$domErr", domErr < 10.0)
        val vehErr = abs(GeoUtils.angleDiffDeg(engine.vehicleHeadingDeg.toDouble(), 90.0))
        assertTrue("Fused heading must point east, err=$vehErr", vehErr < 10.0)
    }

    @Test
    fun testTurnSuppressesSidewaysInertiaFling() {
        // Coordinated right turn at 10 m/s, 30 deg/s → centripetal ≈ 5.2 m/s².
        // A lateral spike (pothole) DURING the turn must not fling the trail:
        // the (v·yawRate) compensation + residual suppression bounds lateralVel.
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 10f, heading = 0f))
        seedMoving(engine, 0f, 10f)

        var timeNanos = 1_000_000_000L
        val yawRateDegS = 30.0
        val yawRateRadS = Math.toRadians(yawRateDegS).toFloat()
        var yaw = 0.0
        repeat(120) { i ->
            timeNanos += 10_000_000L
            yaw += yawRateDegS * 0.01
            // 6 m/s² lateral spike for samples 40..59, else clean coordinated turn.
            val spikeE = if (i in 40 until 60) 6.0 else 0.0
            engine.processImu(imu(timeNanos, yawDeg = yaw, worldE = spikeE, gyroZ = -yawRateRadS))
        }

        assertTrue(
            "Turn spike must stay bounded, lateralVel=${engine.lateralVel}",
            abs(engine.lateralVel) < 1.5
        )
        assertTrue("Forward speed must survive the turn", engine.forwardSpeed > 5.0)
        assertTrue("Yaw-rate tracker must see the turn", abs(engine.yawRateDegS) > 10.0)
    }

    @Test
    fun testTrailMemorySnapsReverseBacktrack() {
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 8f, heading = 90f))
        seedMoving(engine, 90f, 8f)

        // Drive east ~24 m so the breadcrumb trail exists.
        var timeNanos = 1_000_000_000L
        repeat(300) {
            timeNanos += 10_000_000L
            engine.processImu(imu(timeNanos, yawDeg = 90.0))
        }
        assertTrue("Trail must be remembered, size=${engine.trailSize}", engine.trailSize >= 5)

        // Query just south of the driven line, heading WEST (reverse/backtrack):
        // resemblance to the old path (opposite direction) must still match.
        val cur = engine.currentState!!
        val enu = GeoUtils.latLonToEnu(cur.latitude, cur.longitude, 16.3067, 80.4365)
        val match = engine.snapToTrail(enu.first - 10.0, enu.second - 3.0, 270f, 6.0)
        assertNotNull("Reverse backtrack must re-snap to the remembered path", match)
        // Lateral error must collapse onto the old centerline (trail north ≈ 0).
        assertTrue(
            "Snapped point must sit on the old line, north=${match!!.y}",
            abs(match.y - 0.0) < 4.0
        )
    }

    @Test
    fun testDebugLogRowIsFullyPopulatedEverySample() {
        val engine = RouteAwareFusionEngine()
        engine.initialize(createInitialState(speed = 6f, heading = 45f))
        seedMoving(engine, 45f, 6f)

        var timeNanos = 1_000_000_000L
        repeat(30) {
            timeNanos += 10_000_000L
            engine.processImu(imu(timeNanos, yawDeg = 45.0))
            val dbg = engine.lastDebug
            assertNotNull("Every IMU sample must emit a debug row", dbg)
            dbg!!
            assertTrue("forwardSpeed must be finite", dbg.forwardSpeedMps.isFinite())
            assertTrue("yawRate must be finite", dbg.yawRateDegS.isFinite())
            assertTrue("lateralVel must be finite", dbg.lateralVelMps.isFinite())
            assertTrue("fused heading must be finite", dbg.fusedHeadingRad.isFinite())
            assertTrue("snapMode must be labeled, got '${dbg.snapMode}'", dbg.snapMode.isNotEmpty())
            assertTrue("stride increment must be finite", dbg.strideLength.isFinite())
            // Output position and debug row must agree (consistent buffers).
            val st = engine.currentState!!
            assertEquals(st.latitude, dbg.latitude, 1e-9)
            assertEquals(st.longitude, dbg.longitude, 1e-9)
            assertEquals(st.timestampNanos, dbg.timestampNanos)
        }
    }
}
