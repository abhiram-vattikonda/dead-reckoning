package com.sih.idr.navigation

import android.hardware.SensorManager
import com.sih.idr.utils.GeoUtils

/**
 * Frame math + v1 vehicle alignment, kept OUT of the DR engine.
 *
 * Conventions (documented once, used everywhere):
 * - Phone frame: Android standard (X right, Y up, Z out of screen).
 * - World frame: ENU — x = East, y = North, z = Up (from the rotation-vector sensor,
 *   i.e. relative to magnetic north on most phones; fine for a drift baseline).
 * - Gravity points DOWN (-z world). Linear accel = raw accel - gravity.
 * - Units: m/s^2, rad/s. Timestep dt comes from sensor timestamps (seconds).
 *
 * V1 ALIGNMENT: we assume the phone is fixed in a mount for the whole run.
 * [beginCalibration] latches the current yaw as the zero reference; later upgrades
 * (automatic gyro/magnetometer vehicle alignment) plug in here without touching
 * the DR engine.
 */
object CoordinateTransformer {

    /** Quaternion (x,y,z,w, phone->world) to 3x3 row-major rotation matrix. */
    fun rotationMatrixFromQuaternion(x: Float, y: Float, z: Float, w: Float): FloatArray {
        val m = FloatArray(9)
        val xx = x * x; val yy = y * y; val zz = z * z
        val xy = x * y; val xz = x * z; val yz = y * z
        val wx = w * x; val wy = w * y; val wz = w * z
        m[0] = 1f - 2f * (yy + zz); m[1] = 2f * (xy - wz);       m[2] = 2f * (xz + wy)
        m[3] = 2f * (xy + wz);       m[4] = 1f - 2f * (xx + zz); m[5] = 2f * (yz - wx)
        m[6] = 2f * (xz - wy);       m[7] = 2f * (yz + wx);      m[8] = 1f - 2f * (xx + yy)
        return m
    }

    /** v = R * v (row-major 3x3). */
    fun transform(vec: FloatArray, r: FloatArray, out: FloatArray) {
        out[0] = r[0] * vec[0] + r[1] * vec[1] + r[2] * vec[2]
        out[1] = r[3] * vec[0] + r[4] * vec[1] + r[5] * vec[2]
        out[2] = r[6] * vec[0] + r[7] * vec[1] + r[8] * vec[2]
    }

    /**
     * Gravity in the PHONE frame from a phone->world rotation matrix.
     * World gravity = (0, 0, -g); phone gravity = R^T * worldGravity.
     */
    fun gravityInPhoneFrame(r: FloatArray, out: FloatArray, g: Float = SensorManager.GRAVITY_EARTH) {
        out[0] = -g * r[6]
        out[1] = -g * r[7]
        out[2] = -g * r[8]
    }

    /** Yaw/pitch/roll in degrees from a rotation matrix (same convention as SensorManager). */
    fun yawPitchRollDeg(r: FloatArray): FloatArray {
        val o = FloatArray(3)
        SensorManager.getOrientation(r, o)
        return floatArrayOf(
            GeoUtils.normalizeHeading(Math.toDegrees(o[0].toDouble()).toFloat()),
            Math.toDegrees(o[1].toDouble()).toFloat(),
            Math.toDegrees(o[2].toDouble()).toFloat()
        )
    }
}

/**
 * V1 mount calibration: latch yaw at START so headings are reported relative to the
 * run start. Later: estimate phone->vehicle rotation automatically.
 */
class VehicleAlignmentModule {
    private var yawOffsetDeg: Float = 0f
    private var calibrated = false

    fun beginCalibration(currentYawDeg: Float) {
        yawOffsetDeg = currentYawDeg
        calibrated = true
    }

    fun vehicleHeadingDeg(worldYawDeg: Float): Float =
        if (calibrated) GeoUtils.normalizeHeading(worldYawDeg - yawOffsetDeg) else worldYawDeg

    fun reset() { calibrated = false; yawOffsetDeg = 0f }
}
