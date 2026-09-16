package com.sih.idr.sensors

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import androidx.core.app.NotificationCompat

/**
 * Priority boost for test sessions — nothing else.
 *
 * All sensor/GPS logic stays in the ViewModel managers; this service exists so
 * Android (especially aggressive OEM skins) does not suspend IMU/GPS delivery
 * when the screen turns off or the app leaves the foreground mid-drive.
 * Started on session START, stopped on STOP. No state, no logic inside.
 *
 * Declared as location-type FGS because the session includes GPS logging; IMU
 * delivery rides along with the foreground priority and is unaffected by the
 * phone's location on/off switch.
 */
class RecordingService : Service() {

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        promote()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        promote()
        return START_NOT_STICKY // never resurrect a dead session blindly
    }

    private fun promote() {
        val mgr = getSystemService(NotificationManager::class.java) ?: return
        mgr.createNotificationChannel(
            NotificationChannel(CHANNEL, "Session recording", NotificationManager.IMPORTANCE_LOW)
        )
        val note = NotificationCompat.Builder(this, CHANNEL)
            .setContentTitle("IDR test recording")
            .setContentText("Logging IMU + GPS for the dead-reckoning session")
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setOngoing(true)
            .build()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            startForeground(NOTIF_ID, note, ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION)
        } else {
            @Suppress("DEPRECATION")
            startForeground(NOTIF_ID, note)
        }
    }

    companion object {
        const val CHANNEL = "idr_recording"
        const val NOTIF_ID = 41
    }
}
