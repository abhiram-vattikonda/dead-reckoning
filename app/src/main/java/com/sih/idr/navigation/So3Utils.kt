package com.sih.idr.navigation

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Allocation-free SO(3) / SE_2(3) math helpers for the IEKF hot path.
 *
 * All matrices are 3×3, stored row-major as FloatArray(9):
 *   [0 1 2]
 *   [3 4 5]
 *   [6 7 8]
 *
 * None of these functions allocate — callers supply output arrays.
 * Use scratch buffers in the engine; never call from concurrent threads without
 * separate scratch instances.
 */
object So3Utils {

    // ---------------------------------------------------------------------------
    // Basic 3×3 matrix ops
    // ---------------------------------------------------------------------------

    /** out = A · B  (3×3 row-major) */
    fun multiply(a: FloatArray, b: FloatArray, out: FloatArray) {
        out[0] = a[0]*b[0] + a[1]*b[3] + a[2]*b[6]
        out[1] = a[0]*b[1] + a[1]*b[4] + a[2]*b[7]
        out[2] = a[0]*b[2] + a[1]*b[5] + a[2]*b[8]
        out[3] = a[3]*b[0] + a[4]*b[3] + a[5]*b[6]
        out[4] = a[3]*b[1] + a[4]*b[4] + a[5]*b[7]
        out[5] = a[3]*b[2] + a[4]*b[5] + a[5]*b[8]
        out[6] = a[6]*b[0] + a[7]*b[3] + a[8]*b[6]
        out[7] = a[6]*b[1] + a[7]*b[4] + a[8]*b[7]
        out[8] = a[6]*b[2] + a[7]*b[5] + a[8]*b[8]
    }

    /** out = Aᵀ (transpose of 3×3 row-major A) */
    fun transpose(a: FloatArray, out: FloatArray) {
        out[0] = a[0]; out[1] = a[3]; out[2] = a[6]
        out[3] = a[1]; out[4] = a[4]; out[5] = a[7]
        out[6] = a[2]; out[7] = a[5]; out[8] = a[8]
    }

    /** out = R · v  where R is 3×3 row-major and v is FloatArray(3) */
    fun rotVec(r: FloatArray, v: FloatArray, out: FloatArray) {
        out[0] = r[0]*v[0] + r[1]*v[1] + r[2]*v[2]
        out[1] = r[3]*v[0] + r[4]*v[1] + r[5]*v[2]
        out[2] = r[6]*v[0] + r[7]*v[1] + r[8]*v[2]
    }

    /** out = Rᵀ · v */
    fun rotVecT(r: FloatArray, v: FloatArray, out: FloatArray) {
        out[0] = r[0]*v[0] + r[3]*v[1] + r[6]*v[2]
        out[1] = r[1]*v[0] + r[4]*v[1] + r[7]*v[2]
        out[2] = r[2]*v[0] + r[5]*v[1] + r[8]*v[2]
    }

    /** Set out to 3×3 identity. */
    fun identity(out: FloatArray) {
        out[0] = 1f; out[1] = 0f; out[2] = 0f
        out[3] = 0f; out[4] = 1f; out[5] = 0f
        out[6] = 0f; out[7] = 0f; out[8] = 1f
    }

    /** Copy 9-float matrix src → dst. */
    fun copy(src: FloatArray, dst: FloatArray) {
        dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2]
        dst[3] = src[3]; dst[4] = src[4]; dst[5] = src[5]
        dst[6] = src[6]; dst[7] = src[7]; dst[8] = src[8]
    }

    // ---------------------------------------------------------------------------
    // SO(3) exponential map  exp(φ) → R  (Rodrigues formula)
    // ---------------------------------------------------------------------------

    /**
     * SO(3) exponential: rotation vector φ (rad) → 3×3 rotation matrix [out].
     * Uses exact Rodrigues formula; falls back to first-order for |φ| < 1e-7.
     */
    fun exp(phi: FloatArray, out: FloatArray) {
        val px = phi[0]; val py = phi[1]; val pz = phi[2]
        val angle = sqrt((px*px + py*py + pz*pz).toDouble()).toFloat()
        if (angle < 1e-7f) {
            // First-order: R ≈ I + [φ]×
            out[0] = 1f;  out[1] = -pz; out[2] = py
            out[3] = pz;  out[4] = 1f;  out[5] = -px
            out[6] = -py; out[7] = px;  out[8] = 1f
            return
        }
        val ax = px / angle; val ay = py / angle; val az = pz / angle
        val s = sin(angle); val c = cos(angle); val mc = 1f - c
        out[0] = c + ax*ax*mc;       out[1] = ax*ay*mc - az*s;  out[2] = ax*az*mc + ay*s
        out[3] = ay*ax*mc + az*s;    out[4] = c + ay*ay*mc;     out[5] = ay*az*mc - ax*s
        out[6] = az*ax*mc - ay*s;    out[7] = az*ay*mc + ax*s;  out[8] = c + az*az*mc
    }

    // ---------------------------------------------------------------------------
    // Skew-symmetric matrix
    // ---------------------------------------------------------------------------

    /**
     * out = [v]× (3×3 skew-symmetric cross-product matrix of v).
     *   [  0  -v2   v1 ]
     *   [  v2   0  -v0 ]
     *   [ -v1   v0   0 ]
     */
    fun skew(v: FloatArray, out: FloatArray) {
        out[0] = 0f;    out[1] = -v[2]; out[2] = v[1]
        out[3] = v[2];  out[4] = 0f;    out[5] = -v[0]
        out[6] = -v[1]; out[7] = v[0];  out[8] = 0f
    }

    // ---------------------------------------------------------------------------
    // SO(3) normalization: Gram-Schmidt (cheap, called every N steps)
    // ---------------------------------------------------------------------------

    /**
     * Re-orthonormalise a 3×3 rotation matrix in-place using Gram–Schmidt.
     * Cheaper than SVD; error is O(ε²) so it is called every ~100 steps to
     * prevent floating-point drift from accumulating.
     */
    fun normalizeInPlace(r: FloatArray) {
        // Column 0: just normalize
        val c0x = r[0]; val c0y = r[3]; val c0z = r[6]
        val n0 = sqrt((c0x*c0x + c0y*c0y + c0z*c0z).toDouble()).toFloat().let { if (it < 1e-8f) 1f else it }
        val e0x = c0x/n0; val e0y = c0y/n0; val e0z = c0z/n0
        // Column 1: orthogonalise w.r.t. e0
        val c1x = r[1]; val c1y = r[4]; val c1z = r[7]
        val dot01 = e0x*c1x + e0y*c1y + e0z*c1z
        val f1x = c1x - dot01*e0x; val f1y = c1y - dot01*e0y; val f1z = c1z - dot01*e0z
        val n1 = sqrt((f1x*f1x + f1y*f1y + f1z*f1z).toDouble()).toFloat().let { if (it < 1e-8f) 1f else it }
        val e1x = f1x/n1; val e1y = f1y/n1; val e1z = f1z/n1
        // Column 2: cross product e0 × e1
        val e2x = e0y*e1z - e0z*e1y
        val e2y = e0z*e1x - e0x*e1z
        val e2z = e0x*e1y - e0y*e1x
        r[0] = e0x; r[3] = e0y; r[6] = e0z
        r[1] = e1x; r[4] = e1y; r[7] = e1z
        r[2] = e2x; r[5] = e2y; r[8] = e2z
    }

    // ---------------------------------------------------------------------------
    // Build rotation matrix from Android rotation-vector quaternion (x,y,z,w)
    // Same logic as CoordinateTransformer but returns FloatArray(9) row-major
    // ---------------------------------------------------------------------------

    fun fromQuaternion(qx: Float, qy: Float, qz: Float, qw: Float, out: FloatArray) {
        val xx = qx*qx; val yy = qy*qy; val zz = qz*qz
        val xy = qx*qy; val xz = qx*qz; val yz = qy*qz
        val wx = qw*qx; val wy = qw*qy; val wz = qw*qz
        out[0] = 1f - 2f*(yy+zz); out[1] = 2f*(xy-wz);       out[2] = 2f*(xz+wy)
        out[3] = 2f*(xy+wz);       out[4] = 1f - 2f*(xx+zz); out[5] = 2f*(yz-wx)
        out[6] = 2f*(xz-wy);       out[7] = 2f*(yz+wx);      out[8] = 1f - 2f*(xx+yy)
    }

    // ---------------------------------------------------------------------------
    // Extract yaw from rotation matrix (world ENU: x=East, y=North, z=Up)
    // R maps phone frame → world; yaw = atan2(R[1,0], R[0,0]) gives azimuth
    // in compass degrees (0=N, CW).
    // ---------------------------------------------------------------------------

    /**
     * Compass yaw (degrees, 0=North, clockwise) from a 3×3 row-major ENU rotation matrix.
     * Uses column 0 of R which encodes the East direction in the world frame.
     */
    fun yawDeg(r: FloatArray): Float {
        // atan2(East-component of forward-col, North-component of forward-col)
        // In ENU R: column 1 is the body Y-axis (forward) expressed in world ENU
        val yawRad = kotlin.math.atan2(r[1].toDouble(), r[4].toDouble()) // atan2(R[0,1], R[1,1])
        var deg = Math.toDegrees(yawRad).toFloat()
        if (deg < 0f) deg += 360f
        return deg
    }

    /** Cross product: out = a × b */
    fun cross(a: FloatArray, b: FloatArray, out: FloatArray) {
        out[0] = a[1]*b[2] - a[2]*b[1]
        out[1] = a[2]*b[0] - a[0]*b[2]
        out[2] = a[0]*b[1] - a[1]*b[0]
    }

    /** Dot product of two 3-vectors. */
    fun dot(a: FloatArray, b: FloatArray): Float = a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

    /** Norm of a 3-vector. */
    fun norm3(v: FloatArray): Float = sqrt((v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).toDouble()).toFloat()
}
