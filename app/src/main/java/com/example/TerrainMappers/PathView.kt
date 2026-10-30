package com.example.TerrainMappers

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Typeface
import android.view.View
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

/** Draws the reconstructed path (east/north metres, north is up) with turns and magnetic anomalies. */
class PathView(ctx: Context) : View(ctx) {
    var result: PathResult? = null
        set(v) { field = v; invalidate() }

    private val d = resources.displayMetrics.density
    private fun paint(color: Int, style: Paint.Style = Paint.Style.FILL, width: Float = 0f) =
        Paint(Paint.ANTI_ALIAS_FLAG).apply { this.color = color; this.style = style; strokeWidth = width }

    private val bg = paint(0xFFF8F9FA.toInt())
    private val grid = paint(0x22000000, Paint.Style.STROKE, 1f)
    private val pathP = paint(0xFF3A7CA5.toInt(), Paint.Style.STROKE, 3f * d).apply {
        strokeJoin = Paint.Join.ROUND; strokeCap = Paint.Cap.ROUND
    }
    private val startP = paint(0xFF2A9D8F.toInt())
    private val endP = paint(0xFFE76F51.toInt())
    private val leftP = paint(0xFF1D4ED8.toInt())
    private val rightP = paint(0xFF9333EA.toInt())
    private val dipP = paint(0xFF8D5524.toInt())
    private val spikeP = paint(0xFFD62828.toInt())
    private val num = paint(Color.WHITE).apply { textSize = 11f * d; textAlign = Paint.Align.CENTER; typeface = Typeface.DEFAULT_BOLD }
    private val label = paint(0xFF333333.toInt()).apply { textSize = 11f * d }
    private val hint = paint(0xFF888888.toInt()).apply { textSize = 14f * d; textAlign = Paint.Align.CENTER }
    private val bar = paint(0xFF333333.toInt(), Paint.Style.STROKE, 2f * d)

    private fun nice(raw: Double): Double {
        for (s in doubleArrayOf(0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0)) if (s >= raw) return s
        return 10000.0
    }

    override fun onDraw(c: Canvas) {
        super.onDraw(c)
        val w = width.toFloat(); val h = height.toFloat()
        c.drawRect(0f, 0f, w, h, bg)
        val r = result
        if (r == null || !r.ok || r.east.size < 2) {
            c.drawText("Path map appears here after you press Stop", w / 2, h / 2, hint)
            return
        }
        var minE = Double.MAX_VALUE; var maxE = -Double.MAX_VALUE; var minN = Double.MAX_VALUE; var maxN = -Double.MAX_VALUE
        for (i in r.east.indices) {
            minE = min(minE, r.east[i]); maxE = max(maxE, r.east[i]); minN = min(minN, r.north[i]); maxN = max(maxN, r.north[i])
        }
        val span = max(max(maxE - minE, maxN - minN), 10.0)
        val cE = (minE + maxE) / 2; val cN = (minN + maxN) / 2
        val pad = 40.0 * d
        val scale = (min(width, height) - 2 * pad) / span                       // pixels per metre
        fun px(e: Double) = (width / 2.0 + (e - cE) * scale).toFloat()
        fun py(n: Double) = (height / 2.0 - (n - cN) * scale).toFloat()

        val step = nice(span / 6.0)
        var g = floor((cE - width / 2.0 / scale) / step) * step
        while (g < cE + width / 2.0 / scale) { c.drawLine(px(g), 0f, px(g), h, grid); g += step }
        g = floor((cN - height / 2.0 / scale) / step) * step
        while (g < cN + height / 2.0 / scale) { c.drawLine(0f, py(g), w, py(g), grid); g += step }

        val p = Path()
        p.moveTo(px(r.east[0]), py(r.north[0]))
        for (i in 1 until r.east.size) p.lineTo(px(r.east[i]), py(r.north[i]))
        c.drawPath(p, pathP)

        for ((k, a) in r.anomalies.withIndex()) {                                // magnetic anomalies as diamonds
            val i = r.anomalyPoint[k].coerceIn(0, r.east.size - 1)
            val x = px(r.east[i]); val y = py(r.north[i]); val s = 8f * d
            val dm = Path().apply { moveTo(x, y - s); lineTo(x + s, y); lineTo(x, y + s); lineTo(x - s, y); close() }
            c.drawPath(dm, if (a.spike) spikeP else dipP)
        }
        for ((k, t) in r.turns.withIndex()) {                                    // numbered turn markers
            val i = r.turnPoint[k].coerceIn(0, r.east.size - 1)
            val x = px(r.east[i]); val y = py(r.north[i])
            c.drawCircle(x, y, 10f * d, if (t.angle > 0) leftP else rightP)
            c.drawText("${k + 1}", x, y + 4f * d, num)
        }
        val s0 = 7f * d
        c.drawCircle(px(r.east[0]), py(r.north[0]), s0 + 1f * d, startP)
        val ex = px(r.east.last()); val ey = py(r.north.last())
        c.drawRect(ex - s0, ey - s0, ex + s0, ey + s0, endP)

        // scale bar, north arrow, legend
        val barPx = (step * scale).toFloat()
        val bx = 14f * d; val by = h - 16f * d
        c.drawLine(bx, by, bx + barPx, by, bar)
        c.drawLine(bx, by - 4f * d, bx, by + 4f * d, bar); c.drawLine(bx + barPx, by - 4f * d, bx + barPx, by + 4f * d, bar)
        c.drawText(if (step < 1) "%.1f m".format(step) else "%.0f m".format(step), bx, by - 8f * d, label)
        c.drawLine(w - 24f * d, 40f * d, w - 24f * d, 14f * d, bar)
        c.drawLine(w - 24f * d, 14f * d, w - 29f * d, 22f * d, bar); c.drawLine(w - 24f * d, 14f * d, w - 19f * d, 22f * d, bar)
        c.drawText("N", w - 28f * d, 54f * d, label)
        c.drawText("green=start  orange=end", 12f * d, 18f * d, label)
        c.drawText("blue=left turn  purple=right turn", 12f * d, 33f * d, label)
        c.drawText("brown/red diamond = |B| dip / spike", 12f * d, 48f * d, label)
    }
}
