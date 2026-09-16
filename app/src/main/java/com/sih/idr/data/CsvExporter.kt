package com.sih.idr.data

import android.content.Context
import com.sih.idr.evaluation.EvaluationManager
import com.sih.idr.navigation.pdr.PdrDebug
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Dumps a session to three CSVs for Python/Jupyter analysis:
 *  - session_<ts>_nav.csv : GPS rows (with matched DR + errors) PLUS DR-only rows
 *    so DR data is never lost when GPS is absent (location OFF / tunnels).
 *  - session_<ts>_imu.csv : raw IMU samples (+ linear-accel + gravity).
 *  - session_<ts>_pdr.csv : engine intermediates (every step + ~10 Hz).
 * Written to the app-specific external files dir (no storage permission needed).
 */
object CsvExporter {

    data class ExportResult(val navCsv: File, val imuCsv: File, val pdrCsv: File)

    fun exportSession(
        context: Context,
        session: TestSession,
        pdrDebug: List<PdrDebug> = emptyList()
    ): ExportResult {
        val dir = File(context.getExternalFilesDir("exports"), "").also { it.mkdirs() }
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date(session.startTimestampMillis))
        val navCsv = File(dir, "session_${stamp}_nav.csv")
        val imuCsv = File(dir, "session_${stamp}_imu.csv")
        val pdrCsv = File(dir, "session_${stamp}_pdr.csv")

        // --- nav CSV ---
        navCsv.bufferedWriter().use { w ->
            w.write("timestamp,gps_lat,gps_lon,gps_speed,gps_bearing,gps_accuracy," +
                "dr_lat,dr_lon,dr_velocity,dr_heading," +
                "position_error_m,distance_travelled_m,drift_percentage\n")
            var dist = 0.0
            var prevLat: Double? = null
            var prevLon: Double? = null
            // DR states sorted by time for nearest-match lookup.
            val drSorted = session.drStates.sortedBy { it.timestampNanos }
            for (g in session.gpsSamples.sortedBy { it.timestampMillis }) {
                if (prevLat != null && prevLon != null) {
                    dist += com.sih.idr.utils.GeoUtils.haversineMeters(
                        prevLat, prevLon, g.latitude, g.longitude)
                }
                prevLat = g.latitude; prevLon = g.longitude
                val match = nearestDr(drSorted, g.timestampMillis)
                val err = if (match != null) EvaluationManager.errorAt(
                    TrajectoryPoint(g.timestampMillis, g.latitude, g.longitude),
                    session.drTrajectory
                ) else Double.NaN
                val drift = com.sih.idr.utils.GeoUtils.driftPercent(err, dist)
                w.write(listOf(
                    g.timestampMillis.toString(),
                    g.latitude.toString(), g.longitude.toString(),
                    (g.speed?.toString() ?: ""), (g.bearing?.toString() ?: ""),
                    (g.accuracy?.toString() ?: ""),
                    (match?.latitude?.toString() ?: ""), (match?.longitude?.toString() ?: ""),
                    (match?.speed?.toString() ?: ""), (match?.headingDeg?.toString() ?: ""),
                    err.toString(), dist.toString(), drift.toString()
                ).joinToString(",") + "\n")
            }
            // DR-only rows: one per decimated DR state (~5 Hz cap, 20k row cap).
            // These carry the experiment when GPS is absent; gps_* and error
            // columns stay empty, distance_travelled_m is the DR path length.
            val states = drSorted
            val stride = maxOf(1, states.size / 20000).let { s ->
                // Aim near ~5 Hz regardless of engine output rate.
                if (states.size > 1) {
                    val spanS = (states.last().timestampNanos - states.first().timestampNanos) / 1e9
                    if (spanS > 1.0) maxOf(s, (states.size / (spanS * 5)).toInt().coerceAtLeast(1)) else s
                } else s
            }
            var drDist = 0.0
            var dPrevLat: Double? = null
            var dPrevLon: Double? = null
            var i = 0
            while (i < states.size) {
                val d = states[i]
                if (dPrevLat != null && dPrevLon != null) {
                    drDist += com.sih.idr.utils.GeoUtils.haversineMeters(
                        dPrevLat, dPrevLon, d.latitude, d.longitude)
                }
                dPrevLat = d.latitude; dPrevLon = d.longitude
                w.write(listOf(
                    (d.timestampNanos / 1_000_000).toString(),
                    "", "", "", "", "",
                    d.latitude.toString(), d.longitude.toString(),
                    d.speed.toString(), d.headingDeg.toString(),
                    "", drDist.toString(), ""
                ).joinToString(",") + "\n")
                i += stride
            }
        }

        // --- raw IMU CSV ---
        imuCsv.bufferedWriter().use { w ->
            w.write("timestamp_nanos,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z," +
                "mag_x,mag_y,mag_z,quat_x,quat_y,quat_z,quat_w," +
                "linear_x,linear_y,linear_z,gravity_x,gravity_y,gravity_z\n")
            for (s in session.imuSamples) {
                w.write(listOf(
                    s.timestampNanos.toString(),
                    s.accelX.toString(), s.accelY.toString(), s.accelZ.toString(),
                    s.gyroX.toString(), s.gyroY.toString(), s.gyroZ.toString(),
                    (s.magnetometerX?.toString() ?: ""), (s.magnetometerY?.toString() ?: ""),
                    (s.magnetometerZ?.toString() ?: ""),
                    (s.quatX?.toString() ?: ""), (s.quatY?.toString() ?: ""),
                    (s.quatZ?.toString() ?: ""), (s.quatW?.toString() ?: ""),
                    (s.linearX?.toString() ?: ""), (s.linearY?.toString() ?: ""),
                    (s.linearZ?.toString() ?: ""),
                    (s.gravityX?.toString() ?: ""), (s.gravityY?.toString() ?: ""),
                    (s.gravityZ?.toString() ?: "")
                ).joinToString(",") + "\n")
            }
        }
        // --- PDR intermediates CSV (every step + ~10 Hz; merge with nav CSV on timestamps in Python) ---
        pdrCsv.bufferedWriter().use { w ->
            w.write("timestamp_nanos,acc_mag,step_detected,stride_length," +
                "gyro_heading_rad,mag_heading_rad,fused_heading_rad," +
                "dr_x_east,dr_y_north,dr_lat,dr_lon\n")
            for (d in pdrDebug) {
                w.write(listOf(
                    d.timestampNanos.toString(),
                    d.accMagnitude.toString(),
                    if (d.stepDetected) "1" else "0",
                    d.strideLength.toString(),
                    d.gyroHeadingRad.toString(),
                    d.magHeadingRad.toString(),
                    d.fusedHeadingRad.toString(),
                    d.xEast.toString(),
                    d.yNorth.toString(),
                    d.latitude.toString(),
                    d.longitude.toString()
                ).joinToString(",") + "\n")
            }
        }
        return ExportResult(navCsv, imuCsv, pdrCsv)
    }

    private fun nearestDr(
        drSorted: List<NavigationState>,
        gpsTimeMillis: Long
    ): NavigationState? {
        if (drSorted.isEmpty()) return null
        var best: NavigationState? = null
        var bestDt = Long.MAX_VALUE
        for (d in drSorted) {
            val dt = kotlin.math.abs(d.timestampNanos / 1_000_000 - gpsTimeMillis)
            if (dt < bestDt) { bestDt = dt; best = d }
        }
        return if (bestDt <= 2000L) best else null
    }
}
