package com.sih.idr.ui

import android.os.SystemClock
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.LocationOn
import androidx.compose.material3.FloatingActionButton
import androidx.compose.material3.Icon
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.toArgb
import androidx.lifecycle.compose.LocalLifecycleOwner
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import com.sih.idr.data.TrajectoryPoint
import org.osmdroid.events.MapListener
import org.osmdroid.events.ScrollEvent
import org.osmdroid.events.ZoomEvent
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Marker
import org.osmdroid.views.overlay.Polyline

/**
 * OpenStreetMap (osmdroid, key-free) GPS-vs-DR map.
 * Blue polyline/marker = GPS ground truth, RED DOTS = one dot per DR sample,
 * green polyline = planned route.
 *
 * [followTarget]: camera animates to it only while [follow] is true. A manual
 * drag/zoom fires [onUserMove] so the map never fights the user. Because osmdroid
 * also emits scroll events for programmatic [animateTo], those are suppressed via
 * a short timestamp guard ([GUARD_MS]) after each programmatic move.
 */
private const val GUARD_MS = 1200L
private const val FOLLOW_THROTTLE_MS = 1500L

private class MapRefs {
    var mapView: MapView? = null
    var gpsLine: Polyline? = null
    var drDots: DotTrailOverlay? = null
    var routeLine: Polyline? = null
    var gpsMarker: Marker? = null
    var drMarker: Marker? = null
    var lastProgrammaticMs: Long = 0L
    var lastFollowAnimMs: Long = 0L
}

@Composable
fun MapComponent(
    gpsPoints: List<TrajectoryPoint>,
    drPoints: List<TrajectoryPoint>,
    gpsPos: TrajectoryPoint?,
    drPos: TrajectoryPoint?,
    plannedRoute: List<TrajectoryPoint> = emptyList(),
    follow: Boolean,
    modifier: Modifier = Modifier,
    onUserMove: () -> Unit = {},
    onToggleFollow: () -> Unit = {}
) {
    val lifecycle = LocalLifecycleOwner.current.lifecycle
    val refs = remember { MapRefs() }
    // Snapshot callbacks so the MapListener always calls the latest lambdas.
    val latestOnUserMove = rememberUpdatedCallback(onUserMove)

    DisposableEffect(lifecycle) {
        val obs = LifecycleEventObserver { _, event ->
            when (event) {
                Lifecycle.Event.ON_RESUME -> refs.mapView?.onResume()
                Lifecycle.Event.ON_PAUSE -> refs.mapView?.onPause()
                else -> Unit
            }
        }
        lifecycle.addObserver(obs)
        onDispose {
            lifecycle.removeObserver(obs)
            refs.mapView?.onDetach()
            refs.mapView = null
        }
    }

    Box(modifier = modifier) {
        AndroidView(
            modifier = Modifier.matchParentSize(),
            factory = { ctx ->
                MapView(ctx).apply {
                    setTileSource(TileSourceFactory.MAPNIK)
                    setMultiTouchControls(true)
                    controller.setZoom(17.0)
                    val start = (gpsPos ?: drPos)?.let { GeoPoint(it.latitude, it.longitude) }
                        ?: GeoPoint(28.6139, 77.2090) // New Delhi fallback until first fix
                    controller.setCenter(start)

                    refs.gpsLine = Polyline().apply {
                        outlinePaint.color = Color(0xFF1A73E8).toArgb()
                        outlinePaint.strokeWidth = 10f
                    }
                    refs.drDots = DotTrailOverlay(
                        colorArgb = Color(0xFFD93025).toArgb(),
                        radiusPx = 9f
                    )
                    refs.routeLine = Polyline().apply {
                        outlinePaint.color = Color(0xFF188038).toArgb()
                        outlinePaint.strokeWidth = 7f
                    }
                    refs.gpsMarker = Marker(this).apply {
                        title = "GPS (ground truth)"
                        setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
                    }
                    refs.drMarker = Marker(this).apply {
                        title = "DR estimate"
                        setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
                    }
                    overlays.add(refs.gpsLine)
                    overlays.add(refs.drDots)
                    overlays.add(refs.routeLine)
                    overlays.add(refs.gpsMarker)
                    overlays.add(refs.drMarker)

                    addMapListener(object : MapListener {
                        override fun onScroll(event: ScrollEvent?): Boolean {
                            if (SystemClock.uptimeMillis() - refs.lastProgrammaticMs > GUARD_MS) {
                                latestOnUserMove()
                            }
                            return false
                        }

                        override fun onZoom(event: ZoomEvent?): Boolean {
                            if (SystemClock.uptimeMillis() - refs.lastProgrammaticMs > GUARD_MS) {
                                latestOnUserMove()
                            }
                            return false
                        }
                    })
                    refs.mapView = this
                }
            },
            update = { mv ->
                refs.gpsLine?.setPoints(gpsPoints.map { GeoPoint(it.latitude, it.longitude) })
                refs.drDots?.points = drPoints.map { GeoPoint(it.latitude, it.longitude) }
                refs.routeLine?.setPoints(plannedRoute.map { GeoPoint(it.latitude, it.longitude) })
                val g = gpsPos?.let { GeoPoint(it.latitude, it.longitude) }
                val d = drPos?.let { GeoPoint(it.latitude, it.longitude) }
                if (g != null) refs.gpsMarker?.position = g
                if (d != null) refs.drMarker?.position = d
                // Hide markers until we have a real position at least once.
                refs.gpsMarker?.isEnabled = g != null
                refs.drMarker?.isEnabled = d != null

                if (follow) {
                    val target = g ?: d
                    val now = SystemClock.uptimeMillis()
                    if (target != null && now - refs.lastFollowAnimMs > FOLLOW_THROTTLE_MS) {
                        refs.lastFollowAnimMs = now
                        refs.lastProgrammaticMs = now
                        mv.controller.animateTo(target)
                    }
                }
                mv.invalidate()
            }
        )
        FloatingActionButton(
            onClick = onToggleFollow,
            modifier = Modifier.align(Alignment.TopEnd).padding(12.dp)
        ) {
            Icon(
                Icons.Filled.LocationOn,
                contentDescription = if (follow) "Following (tap to free map)" else "Follow vehicle"
            )
        }
    }
}

/** Like rememberUpdatedState but returns a stable () -> Unit callback. */
@Composable
private fun rememberUpdatedCallback(callback: () -> Unit): () -> Unit {
    val state = androidx.compose.runtime.rememberUpdatedState(callback)
    return remember { { state.value() } }
}
