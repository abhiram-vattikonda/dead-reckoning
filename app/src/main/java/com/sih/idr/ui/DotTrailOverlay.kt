package com.sih.idr.ui

import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Point
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Overlay

/**
 * Renders a trail as one dot per sample instead of a connected polyline, so
 * each plotted point corresponds to exactly one navigation output (10 Hz).
 * Gaps/clumps in the dots are real information: sampling dropouts show as gaps,
 * slow/standstill periods as dense clusters — a line would hide both.
 *
 * Only the most recent [maxPoints] are drawn (full history stays in the session/CSV).
 */
class DotTrailOverlay(
    colorArgb: Int,
    private val radiusPx: Float = 9f,
    private val maxPoints: Int = 10_000
) : Overlay() {

    var points: List<GeoPoint> = emptyList()

    private val paint = Paint().apply {
        isAntiAlias = true
        style = Paint.Style.FILL
        color = colorArgb
    }
    private val reuse = Point()

    override fun draw(canvas: Canvas?, mapView: MapView?, shadow: Boolean) {
        if (shadow || canvas == null || mapView == null) return
        val projection = mapView.projection ?: return
        val pts = points
        val start = (pts.size - maxPoints).coerceAtLeast(0)
        for (i in start until pts.size) {
            projection.toPixels(pts[i], reuse)
            canvas.drawCircle(reuse.x.toFloat(), reuse.y.toFloat(), radiusPx, paint)
        }
    }
}
