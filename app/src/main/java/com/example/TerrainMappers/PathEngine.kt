package com.example.TerrainMappers

import java.util.Locale
import kotlin.math.*

// =====================================================================================================
//  Data types
// =====================================================================================================
class Vec(val x: DoubleArray, val y: DoubleArray, val z: DoubleArray) { val n get() = x.size }
class RawSeries(val t: DoubleArray, val x: DoubleArray, val y: DoubleArray, val z: DoubleArray)
class Recording(val gyro: RawSeries?, val grav: RawSeries?, val accel: RawSeries?, val mag: RawSeries?)

/** Algorithm 2.0 parameters (identical defaults to Alogrithm_2_0.py v1.1). */
data class Cfg(
    val fs: Double = 50.0,
    val cutoffHz: Double = 0.4,
    val adaptiveCutoff: Boolean = true,
    val cutoffMin: Double = 0.25,
    val cutoffMax: Double = 0.5,
    val removeBias: Boolean = true,
    val trigger: Double = 0.25,
    val tail: Double = 0.15,
    val mergeS: Double = 1.5,
    val minAngle: Double = 20.0,
    val handlingDeg: Double = 65.0,
    val handlingPadS: Double = 0.0,
    val walkTiltDeg: Double = 35.0,
    val windowGuardS: Double = 1.0,
    val axisPadS: Double = 3.0,
)

data class Options(
    val strideM: Double = 1.4,            // metres per gait cycle (= 2 steps) when the accelerometer stride model is off
    val useAccelStride: Boolean = true,   // Weinberg stride from the accelerometer, per gait cycle
    val startBearingDeg: Double = 0.0,    // compass bearing of the first walking direction (0 = map "up" is the start direction)
    val cfg: Cfg = Cfg(),
)

data class Turn(
    val t0: Double, val t1: Double, val angle: Double, val angleDyn: Double, val peak: Double,
    val duration: Double, val gravityMove: Double, val touchesEdge: Boolean, val confidence: Double,
    val reason: String? = null, val magAngle: Double = Double.NaN,
)

data class Anomaly(val tStart: Double, val tEnd: Double, val kind: String, val extreme: Double, val confidence: Double, val spike: Boolean)

class PathResult(
    val ok: Boolean,
    val report: String,
    val east: DoubleArray = DoubleArray(0),          // path points, index 0 = start
    val north: DoubleArray = DoubleArray(0),
    val cycleTimes: DoubleArray = DoubleArray(0),    // time of path point k+1
    val turns: List<Turn> = emptyList(),
    val rejected: List<Turn> = emptyList(),
    val turnPoint: IntArray = IntArray(0),           // path-point index of each accepted turn
    val anomalies: List<Anomaly> = emptyList(),
    val anomalyPoint: IntArray = IntArray(0),
)

// =====================================================================================================
//  Engine
// =====================================================================================================
object PathEngine {
    private fun f(fmt: String, vararg a: Any?) = String.format(Locale.US, fmt, *a)
    private fun trapz(y: DoubleArray, i0: Int, i1: Int, dx: Double): Double {
        var s = 0.0
        for (i in i0 until i1) s += 0.5 * (y[i] + y[i + 1]) * dx
        return s
    }
    private fun dot(a: Vec, i: Int, j: Int) = a.x[i] * a.x[j] + a.y[i] * a.y[j] + a.z[i] * a.z[j]
    private fun acosDeg(c: Double) = Math.toDegrees(acos(c.coerceIn(-1.0, 1.0)))

    // ------------------------------------------------------------------ loading / resampling to a common grid
    private fun clean(s: RawSeries?): RawSeries? {
        if (s == null || s.t.size < 10) return null
        val keep = ArrayList<Int>(s.t.size)
        var last = Double.NEGATIVE_INFINITY
        for (i in s.t.indices) {
            if (s.t[i] > last && s.x[i].isFinite() && s.y[i].isFinite() && s.z[i].isFinite()) { keep.add(i); last = s.t[i] }
        }
        if (keep.size < 10) return null
        return RawSeries(DoubleArray(keep.size) { s.t[keep[it]] }, DoubleArray(keep.size) { s.x[keep[it]] },
            DoubleArray(keep.size) { s.y[keep[it]] }, DoubleArray(keep.size) { s.z[keep[it]] })
    }

    private fun resample(t: DoubleArray, s: RawSeries) =
        Vec(Dsp.interp(t, s.t, s.x), Dsp.interp(t, s.t, s.y), Dsp.interp(t, s.t, s.z))

    private class Aligned(val t: DoubleArray, val w: Vec, val u: Vec, val acc: Vec?, val mag: Vec?, val gravityDerived: Boolean)

    private fun align(rec: Recording, fs: Double): Aligned? {
        val g = clean(rec.gyro) ?: return null
        val r = clean(rec.grav); val ac = clean(rec.accel); val mg = clean(rec.mag)
        if (r == null && ac == null) return null
        val all = listOfNotNull(g, r, ac, mg)
        val t0 = all.maxOf { it.t[0] }
        val t1 = all.minOf { it.t[it.t.size - 1] }
        val n = ceil((t1 - t0) * fs).toInt()
        if (n < 10) return null
        val t = DoubleArray(n) { t0 + it / fs }
        val acc = ac?.let { resample(t, it) }
        var gv: Vec; var derived = false
        if (r != null) gv = resample(t, r) else {
            derived = true                                           // no gravity sensor -> low-pass the accelerometer at 2.5 Hz (keeps the ~1 Hz gait swing Algorithm 2.0 needs)
            gv = Vec(Dsp.lowpass(acc!!.x, fs, 2.5), Dsp.lowpass(acc.y, fs, 2.5), Dsp.lowpass(acc.z, fs, 2.5))
        }
        val u = Vec(DoubleArray(n), DoubleArray(n), DoubleArray(n))
        for (i in 0 until n) {
            val nr = sqrt(gv.x[i] * gv.x[i] + gv.y[i] * gv.y[i] + gv.z[i] * gv.z[i]).let { if (it == 0.0) 1.0 else it }
            u.x[i] = gv.x[i] / nr; u.y[i] = gv.y[i] / nr; u.z[i] = gv.z[i] / nr
        }
        return Aligned(t, resample(t, g), u, acc, mg?.let { resample(t, it) }, derived)
    }

    // ------------------------------------------------------------------ Algorithm 2.0: helpers
    private fun gravityPc(u: Vec, a: Int, b: Int): DoubleArray {
        val n = b - a
        var mx = 0.0; var my = 0.0; var mz = 0.0
        for (i in a until b) { mx += u.x[i]; my += u.y[i]; mz += u.z[i] }
        mx /= n; my /= n; mz /= n
        var cxx = 0.0; var cxy = 0.0; var cxz = 0.0; var cyy = 0.0; var cyz = 0.0; var czz = 0.0
        for (i in a until b) {
            val x = u.x[i] - mx; val y = u.y[i] - my; val z = u.z[i] - mz
            cxx += x * x; cxy += x * y; cxz += x * z; cyy += y * y; cyz += y * z; czz += z * z
        }
        var v0 = 1.0; var v1 = 0.7; var v2 = 0.3                    // power iteration = first right singular vector
        repeat(200) {
            val w0 = cxx * v0 + cxy * v1 + cxz * v2
            val w1 = cxy * v0 + cyy * v1 + cyz * v2
            val w2 = cxz * v0 + cyz * v1 + czz * v2
            val nr = sqrt(w0 * w0 + w1 * w1 + w2 * w2).let { if (it == 0.0) 1.0 else it }
            v0 = w0 / nr; v1 = w1 / nr; v2 = w2 / nr
        }
        return DoubleArray(n) { (u.x[a + it] - mx) * v0 + (u.y[a + it] - my) * v1 + (u.z[a + it] - mz) * v2 }
    }

    /** Dominant gait-cycle frequency of the gravity oscillation inside [fLo, fHi], or null if no clear peak. */
    private fun strideFrequency(u: Vec, a: Int, b: Int, fs: Double, fHi: Double = 1.3, minSeconds: Double = 4.0): Double? {
        if (b - a < minSeconds * fs) return null
        val sig = gravityPc(u, a, b)
        val w = Dsp.hann(sig.size)
        for (i in sig.indices) sig[i] *= w[i]
        val nfft = Dsp.nextPow2(max(4096, sig.size))
        val p = Dsp.powerSpectrum(sig, nfft)
        val df = fs / nfft
        val k0 = ceil(0.5 / df).toInt(); val k1 = floor(fHi / df).toInt()
        if (k1 <= k0) return null
        var kBest = k0
        for (k in k0..k1) if (p[k] > p[kBest]) kBest = k
        val med = Dsp.median(DoubleArray(k1 - k0 + 1) { p[k0 + it] })
        return if (p[kBest] > 5 * med) kBest * df else null
    }

    /** Longest stretch with gravity within walkTiltDeg of the dominant direction (phone resting in the pocket). */
    private fun walkingWindow(u: Vec, cfg: Cfg): Pair<Int, Int> {
        val n = u.n; val fs = cfg.fs
        val cosmax = cos(Math.toRadians(cfg.walkTiltDeg))
        val step = max(1, (fs / 2).toInt())
        var bestC = 0; var bestCnt = -1
        var c = 0
        while (c < n) {                                              // counting on every 5th sample keeps this O(n^2/250)
            var cnt = 0; var j = 0
            while (j < n) { if (dot(u, c, j) > cosmax) cnt++; j += 5 }
            if (cnt > bestCnt) { bestCnt = cnt; bestC = c }
            c += step
        }
        var rx = 0.0; var ry = 0.0; var rz = 0.0
        for (j in 0 until n) if (dot(u, bestC, j) > cosmax) { rx += u.x[j]; ry += u.y[j]; rz += u.z[j] }
        val rn = sqrt(rx * rx + ry * ry + rz * rz).let { if (it == 0.0) 1.0 else it }
        rx /= rn; ry /= rn; rz /= rn
        val tilt = DoubleArray(n) { acosDeg(u.x[it] * rx + u.y[it] * ry + u.z[it] * rz) }
        val k = fs.toInt()
        val pre = DoubleArray(n + 1); for (i in 0 until n) pre[i + 1] = pre[i] + tilt[i]
        // numpy "same" convolution with a length-k box: window [i-k/2, i+(k-1)/2], zero padded
        val ok = BooleanArray(n) { i ->
            val lo = max(0, i - k / 2); val hi = min(n - 1, i + (k - 1) / 2)
            (pre[hi + 1] - pre[lo]) / k < cfg.walkTiltDeg
        }
        var bestS = -1; var bestE = -1; var i = 0
        while (i < n) {
            if (ok[i]) {
                var j = i; while (j < n && ok[j]) j++
                if (bestS < 0 || j - i > bestE - bestS) { bestS = i; bestE = j }
                i = j
            } else i++
        }
        if (bestS < 0) return 0 to 0
        val g = (cfg.windowGuardS * fs).toInt()
        return if (bestE - bestS > 2 * g) (bestS + g) to (bestE - g) else 0 to 0
    }

    private class Info(var a: Int, var b: Int, var fStride: Double?, var cutoff: Double, var bias: Double, val rejected: MutableList<Turn> = ArrayList())
    private class Seg(val c0: Int, val c1: Int, val sg: Int)
    private class Span(val r: DoubleArray, val i0: Int, val i1: Int)

    private fun confidence(ang: Double, angDyn: Double, peak: Double, gm: Double, edge: Boolean, cfg: Cfg): Double {
        val cPeak = ((peak - cfg.trigger) / cfg.trigger).coerceIn(0.0, 1.0)
        val cAng = ((abs(ang) - cfg.minAngle) / 25).coerceIn(0.0, 1.0)
        val cGrav = (1 - max(gm - 15, 0.0) / (cfg.handlingDeg - 15)).coerceIn(0.0, 1.0)
        val cAxis = (1 - abs(ang - angDyn) / max(abs(ang), cfg.minAngle)).coerceIn(0.0, 1.0)
        return 0.25 * (cPeak + cAng + cGrav + cAxis) * (if (edge) 0.6 else 1.0)
    }

    // ------------------------------------------------------------------ Algorithm 2.0: turn detection
    private fun detectTurns(t: DoubleArray, w: Vec, u: Vec, cfg: Cfg): Pair<Info, List<Turn>> {
        val fs = cfg.fs; val n = t.size
        val (a, b) = walkingWindow(u, cfg)
        val info = Info(a, b, null, cfg.cutoffHz, 0.0)
        if (b - a < 2 * fs) return info to emptyList()

        val fstr = strideFrequency(u, a, b, fs)
        val fc = if (cfg.adaptiveCutoff && fstr != null) (0.5 * fstr).coerceIn(cfg.cutoffMin, cfg.cutoffMax) else cfg.cutoffHz
        val raw = DoubleArray(n) { w.x[it] * u.x[it] + w.y[it] * u.y[it] + w.z[it] * u.z[it] }
        var s = Dsp.lowpass(raw, fs, fc)
        var bias = 0.0
        if (cfg.removeBias) {
            val quiet = (a until b).map { s[it] }.filter { abs(it) < cfg.trigger }
            if (quiet.size > 2 * fs) bias = Dsp.median(quiet.toDoubleArray())
        }
        if (bias != 0.0) s = DoubleArray(n) { s[it] - bias }
        info.fStride = fstr; info.cutoff = fc; info.bias = bias

        val segs = ArrayList<Seg>()
        var lastEnd = -1
        for (i in a until b) {
            if (abs(s[i]) > cfg.trigger) {
                val sg = if (s[i] > 0) 1 else -1
                if (segs.isNotEmpty() && segs.last().sg == sg && (i - lastEnd) / fs < cfg.mergeS) {
                    val old = segs.removeAt(segs.size - 1); segs.add(Seg(old.c0, i, sg))
                } else segs.add(Seg(i, i, sg))
                lastEnd = i
            }
        }

        val pad = (cfg.axisPadS * fs).toInt()
        val hp = (cfg.handlingPadS * fs).toInt()

        fun measure(sg: Seg): Span {
            var pk = sg.c0
            for (i in sg.c0..sg.c1) if (sg.sg * s[i] > sg.sg * s[pk]) pk = i
            val lo = max(a, pk - pad); val hi = min(b, pk + pad)
            var ax = 0.0; var ay = 0.0; var az = 0.0
            for (i in lo until hi) { ax += u.x[i]; ay += u.y[i]; az += u.z[i] }
            val nr = sqrt(ax * ax + ay * ay + az * az).let { if (it == 0.0) 1.0 else it }
            ax /= nr; ay /= nr; az /= nr
            val proj = DoubleArray(n) { w.x[it] * ax + w.y[it] * ay + w.z[it] * az }
            val r = Dsp.lowpass(proj, fs, fc)
            if (bias != 0.0) for (i in 0 until n) r[i] -= bias
            var i0 = pk; var i1 = pk
            while (i0 > a && sg.sg * r[i0] > cfg.tail) i0--
            while (i1 < b - 1 && sg.sg * r[i1] > cfg.tail) i1++
            return Span(r, i0, i1)
        }

        val spans = ArrayList<Span>()
        for (sg in segs) spans.add(measure(sg))
        while (true) {                                               // merge turns whose extended spans overlap
            var merged = false
            for (k in 0 until segs.size - 1) {
                if (segs[k].sg == segs[k + 1].sg && spans[k + 1].i0 <= spans[k].i1) {
                    segs[k] = Seg(segs[k].c0, segs[k + 1].c1, segs[k].sg)
                    segs.removeAt(k + 1); spans.removeAt(k + 1)
                    spans[k] = measure(segs[k])
                    merged = true; break
                }
            }
            if (!merged) break
        }

        val kept = ArrayList<Turn>()
        for ((idx, _) in segs.withIndex()) {
            val sp = spans[idx]; val i0 = sp.i0; val i1 = sp.i1; val r = sp.r
            val ang = Math.toDegrees(trapz(r, i0, i1, 1 / fs))
            val angDyn = Math.toDegrees(trapz(s, i0, i1, 1 / fs))
            var peak = 0.0; for (i in i0..i1) peak = max(peak, abs(r[i]))
            val lo = max(a, i0 - hp); val hi = min(b - 1, i1 + hp)
            var gm = 0.0; for (i in lo..hi) gm = max(gm, acosDeg(dot(u, i, i0)))
            val edge = i0 <= a || i1 >= b - 1
            val reason = if (abs(ang) < cfg.minAngle) "angle below MIN_ANGLE"
                         else if (gm > cfg.handlingDeg) "gravity moved (phone handled)" else null
            val turn = Turn(t[i0], t[i1], ang, angDyn, peak, t[i1] - t[i0], gm, edge,
                confidence(ang, angDyn, peak, gm, edge, cfg), reason)
            if (reason != null) info.rejected.add(turn) else kept.add(turn)
        }
        return info to kept
    }

    private fun headingSeries(t: DoubleArray, a: Int, b: Int, turns: List<Turn>): DoubleArray {
        val tw = t.copyOfRange(a, b)
        val head = DoubleArray(tw.size)
        var cum = 0.0
        for (tr in turns.sortedBy { it.t0 }) {
            val i0 = Dsp.searchSorted(tw, tr.t0); val i1 = Dsp.searchSorted(tw, tr.t1)
            val d = Math.toRadians(tr.angle)
            val m = max(i1 - i0, 1)
            for (j in 0 until (i1 - i0)) head[i0 + j] = cum + (if (m == 1) 0.0 else d * j / (m - 1))
            cum += d
            for (j in i1 until head.size) head[j] = cum
        }
        return head
    }

    // ------------------------------------------------------------------ accelerometer (filtered pipeline, Weinberg stride)
    private class AccelResult(
        val label: String, val windowsText: String, val stepT: DoubleArray, val stepLen: DoubleArray,
        val clipped: Int, val hampel: Int, val note: String?,
    )

    private const val MAX_PLAUSIBLE_ACCEL = 60.0
    private const val WALK_CENTROID = 2.2
    private const val JOG_CENTROID = 4.2
    private const val WALK_CLEAR = 3.0
    private const val JOG_CLEAR = 8.0
    private const val SPRINT_PEAK_MIN = 45.0
    private const val STAIRS_DRIFT_SLOPE_MIN = 1.0
    private const val WEINBERG_K = 0.5

    private fun accelAnalysis(t: DoubleArray, acc: Vec, fs: Double, winA: Int, winB: Int, fCadence: Double?): AccelResult {
        val n = t.size
        val lx = Dsp.highpass(acc.x, fs, 0.3); val ly = Dsp.highpass(acc.y, fs, 0.3); val lz = Dsp.highpass(acc.z, fs, 0.3)
        val mag = DoubleArray(n) { sqrt(lx[it] * lx[it] + ly[it] * ly[it] + lz[it] * lz[it]) }
        var clipped = 0
        for (i in 0 until n) if (mag[i] > MAX_PLAUSIBLE_ACCEL) { clipped++; mag[i] = MAX_PLAUSIBLE_ACCEL }
        val (clean, nOut) = Dsp.hampel(mag)
        val band = Dsp.bandpass(clean, fs, 0.5, 3.5)
        val drift = Dsp.lowpass(clean, fs, 0.3)

        val cadenceHint = when {
            fCadence == null -> null
            fCadence < 1.1 -> "WALK_CADENCE"
            fCadence > 1.3 -> "JOG_CADENCE"
            else -> null
        }

        val W = (4.0 * fs).toInt(); val hop = W / 2
        val starts = ArrayList<Int>()
        if (n < W) starts.add(0) else { var s0 = 0; while (s0 + W <= n) { starts.add(s0); s0 += hop } }
        val labels = ArrayList<String>(); val sb = StringBuilder()
        val peakIdx = sortedSetOf<Int>()
        var stairsNote: String? = null
        for (s0 in starts) {
            val e0 = min(n, s0 + W)
            if (e0 - s0 < 5) continue
            val segBand = band.copyOfRange(s0, e0)
            val segClean = clean.copyOfRange(s0, e0)
            val pk = Dsp.findPeaks(segBand, max(1, (fs * 0.25).toInt()), max(0.15, 0.5 * Dsp.std(segBand)))
            val stationary = segClean.count { it < 0.3 }.toDouble() / segClean.size
            var label: String; var note: String? = null
            val meanPeak = if (pk.isNotEmpty()) pk.map { segClean[it] }.average() else 0.0
            if (stationary >= 0.8) label = "STANDING_STILL" else {
                // slope of the <0.3 Hz drift component (m/s^2 per second) via least squares
                val tt = DoubleArray(e0 - s0) { (it) / fs }; val dd = drift.copyOfRange(s0, e0)
                val mt = tt.average(); val md = dd.average()
                var num = 0.0; var den = 0.0
                for (i in tt.indices) { num += (tt[i] - mt) * (dd[i] - md); den += (tt[i] - mt) * (tt[i] - mt) }
                val slope = if (den > 0) num / den else 0.0
                val inWalkNow = t[s0] >= t[winA] && t[min(e0, n) - 1] <= t[min(winB, n) - 1]
                if (abs(slope) >= STAIRS_DRIFT_SLOPE_MIN) { note = "possible stairs"; if (inWalkNow) stairsNote = note }
                label = when {
                    meanPeak >= SPRINT_PEAK_MIN -> "SPRINT"
                    meanPeak <= WALK_CLEAR -> "WALK"
                    meanPeak >= JOG_CLEAR -> "JOG"
                    abs(meanPeak - WALK_CENTROID) <= abs(meanPeak - JOG_CENTROID) -> "WALK" else -> "JOG"
                }
                if (cadenceHint == "WALK_CADENCE" && (label == "JOG" || label == "SPRINT")) { note = (note?.plus("; ") ?: "") + "cadence-overridden from $label"; label = "WALK" }
                else if (cadenceHint == "JOG_CADENCE" && label == "WALK") { note = (note?.plus("; ") ?: "") + "cadence-overridden from WALK"; label = "JOG" }
                for (p in pk) peakIdx.add(p + s0)
            }
            labels.add(label)
            val inWalk = t[s0] >= t[winA] && t[min(e0, n) - 1] <= t[min(winB, n) - 1]
            if (inWalk) sb.append(f("  [%6.1fs-%6.1fs] %-14s peaks=%2d meanpeak=%5.2f%s\n", t[s0] - t[0], t[e0 - 1] - t[0], label, pk.size, meanPeak, note?.let { "  <- $it" } ?: ""))
        }
        // near-duplicate peaks from overlapping windows -> keep one (the higher)
        val uniq = ArrayList<Int>()
        for (p in peakIdx) {
            if (uniq.isNotEmpty() && p - uniq.last() < (fs * 0.25).toInt()) { if (clean[p] > clean[uniq.last()]) uniq[uniq.size - 1] = p }
            else uniq.add(p)
        }
        val half = max(1, (fs * 0.3).toInt())
        val stepT = DoubleArray(uniq.size) { t[uniq[it]] }
        val stepLen = DoubleArray(uniq.size) {
            val lo = max(0, uniq[it] - half); val hi = min(n, uniq[it] + half)
            var mx = Double.NEGATIVE_INFINITY; var mn = Double.POSITIVE_INFINITY
            for (i in lo until hi) { mx = max(mx, clean[i]); mn = min(mn, clean[i]) }
            WEINBERG_K * max(mx - mn, 1e-3).pow(0.25)
        }
        // overall label among windows overlapping the walking window
        val inW = labels.filterIndexed { k, _ -> val s0 = starts[k]; t[s0] >= t[winA] - 4.0 && t[min(n, s0 + W) - 1] <= t[min(winB, n) - 1] + 4.0 }
        val pool = if (inW.isNotEmpty()) inW else labels
        val dominant = pool.groupingBy { it }.eachCount().maxByOrNull { it.value }?.key ?: "UNKNOWN"
        return AccelResult(dominant, sb.toString(), stepT, stepLen, clipped, nOut, stairsNote)
    }

    // ------------------------------------------------------------------ magnetometer (hard-iron calibration, Hampel, tilt-compensated azimuth, |B| anomalies)
    private const val MAG_BASELINE_UT = 41.5       // Vithura-local baselines, as in magnetometer_filtered.py
    private const val MAG_DIP_UT = 32.0
    private const val MAG_SPIKE_UT = 52.0
    private const val ANOMALY_MIN_DURATION_S = 0.3

    private class MagResult(
        val meanB: Double, val bias: DoubleArray, val nOutXyz: Int, val nOutB: Int,
        val turnAngles: DoubleArray, val switchbacks: Int, val anomalies: List<Anomaly>, val refAxis: Int, val calibrated: Boolean,
    )

    private fun circMean(az: DoubleArray, i0: Int, i1: Int): Double {
        var s = 0.0; var c = 0.0
        for (i in i0 until i1) { s += sin(az[i]); c += cos(az[i]) }
        return atan2(s, c)
    }
    private fun wrapPi(d: Double): Double { var x = d % (2 * PI); if (x > PI) x -= 2 * PI; if (x < -PI) x += 2 * PI; return x }

    private fun magAnalysis(t: DoubleArray, mag: Vec, u: Vec, fs: Double, a: Int, b: Int, turns: List<Turn>): MagResult {
        val n = t.size
        val raw = arrayOf(mag.x, mag.y, mag.z)
        val meanRaw = DoubleArray(n) { sqrt(raw[0][it].pow(2) + raw[1][it].pow(2) + raw[2][it].pow(2)) }.average()
        // One-shot min/max hard-iron centring - but only when the phone tumbled through enough orientations that every axis
        // reaches +-60% of the field strength; otherwise min/max is NOT the sphere centre and would corrupt the heading.
        val covered = (0..2).all { raw[it].min() <= -0.6 * meanRaw && raw[it].max() >= 0.6 * meanRaw }
        val bias = DoubleArray(3) { if (covered) (raw[it].max() + raw[it].min()) / 2.0 else 0.0 }
        var nOut = 0
        val cor = Array(3) { ax -> val (c, k) = Dsp.hampel(DoubleArray(n) { raw[ax][it] - bias[ax] }); nOut += k; c }
        val (bAbs, nOutB) = Dsp.hampel(DoubleArray(n) { sqrt(raw[0][it].pow(2) + raw[1][it].pow(2) + raw[2][it].pow(2)) })

        // reference body axis = the phone axis that stays closest to horizontal while walking
        val meanU = doubleArrayOf((a until b).map { u.x[it] }.average(), (a until b).map { u.y[it] }.average(), (a until b).map { u.z[it] }.average())
        val refAxis = (0..2).minByOrNull { abs(meanU[it]) } ?: 0
        val sm = Array(3) { Dsp.lowpass(cor[it], fs, 2.0, 4) }                         // 2 Hz, 4th order heading smoothing
        val cz = DoubleArray(n); val sz = DoubleArray(n)
        for (i in 0 until n) {
            // East = m x up, North = up x East (Android getRotationMatrix convention); azimuth = bearing of the reference axis
            val hx = sm[1][i] * u.z[i] - sm[2][i] * u.y[i]
            val hy = sm[2][i] * u.x[i] - sm[0][i] * u.z[i]
            val hz = sm[0][i] * u.y[i] - sm[1][i] * u.x[i]
            val hn = sqrt(hx * hx + hy * hy + hz * hz)
            if (hn < 1e-9) { cz[i] = if (i > 0) cz[i - 1] else 1.0; sz[i] = if (i > 0) sz[i - 1] else 0.0; continue }
            val ex = hx / hn; val ey = hy / hn; val ez = hz / hn
            val nx = u.y[i] * ez - u.z[i] * ey; val ny = u.z[i] * ex - u.x[i] * ez; val nz = u.x[i] * ey - u.y[i] * ex
            val e = doubleArrayOf(ex, ey, ez); val nn = doubleArrayOf(nx, ny, nz)
            val az = atan2(e[refAxis], nn[refAxis])                                   // clockwise from magnetic north
            cz[i] = cos(az); sz[i] = sin(az)
        }
        val czS = Dsp.lowpass(cz, fs, 2.0, 4); val szS = Dsp.lowpass(sz, fs, 2.0, 4)
        val az = DoubleArray(n) { atan2(szS[it], czS[it]) }

        // per-turn cross-check: left turn => azimuth decreases
        fun idx(time: Double) = ((time - t[0]) * fs).roundToInt().coerceIn(a, b)
        val turnAngles = DoubleArray(turns.size) { k ->
            val tr = turns[k]
            val b0 = idx(tr.t0 - 1.5); val b1 = idx(tr.t0 - 0.3); val a0 = idx(tr.t1 + 0.3); val a1 = idx(tr.t1 + 1.5)
            if (b1 - b0 < 10 || a1 - a0 < 10) Double.NaN
            else -Math.toDegrees(wrapPi(circMean(az, a0, a1) - circMean(az, b0, b1)))
        }

        // ~180 deg switchbacks (>=150 deg within 2.5 s), 2 s refractory, walking window only
        var sw = 0; var lastT = -1e9
        val win = (2.5 * fs).toInt()
        for (i in max(a, win) until b) {
            if (t[i] - lastT < 2.0) continue
            val d = Math.toDegrees(wrapPi(az[i] - az[i - win]))
            if (abs(d) >= 150.0) { sw++; lastT = t[i] }
        }

        // |B| anomalies (walking window only)
        val events = ArrayList<Anomaly>()
        val minSamples = max(1, (ANOMALY_MIN_DURATION_S * fs).toInt())
        for (spike in listOf(false, true)) {
            val ref = if (spike) MAG_SPIKE_UT else MAG_DIP_UT
            var i = a
            while (i < b) {
                val hit = if (spike) bAbs[i] > MAG_SPIKE_UT else bAbs[i] < MAG_DIP_UT
                if (hit) {
                    var j = i
                    while (j < b && (if (spike) bAbs[j] > MAG_SPIKE_UT else bAbs[j] < MAG_DIP_UT)) j++
                    if (j - i >= minSamples) {
                        var ex = bAbs[i]
                        for (k in i until j) ex = if (spike) max(ex, bAbs[k]) else min(ex, bAbs[k])
                        val conf = (abs(ex - ref) / ref + (j - i).toDouble() / (minSamples * 3)).coerceIn(0.0, 1.0)
                        events.add(Anomaly(t[i], t[j - 1], if (spike) "metal-object spike" else "rebar/concrete-slab dip", ex, conf, spike))
                    }
                    i = j
                } else i++
            }
        }
        return MagResult(bAbs.average(), bias, nOut, nOutB, turnAngles, sw, events.sortedBy { it.tStart }, refAxis, covered)
    }

    // ------------------------------------------------------------------ main entry
    fun analyze(rec: Recording, opt: Options = Options()): PathResult {
        val cfg = opt.cfg
        val al = align(rec, cfg.fs)
            ?: return PathResult(false, "Not enough data. Need Gyroscope plus Gravity (or Accelerometer) with a few seconds of overlap.")
        val t = al.t; val fs = cfg.fs
        val (info, turns) = detectTurns(t, al.w, al.u, cfg)
        val a = info.a; val b = info.b
        if (b - a < 2 * fs)
            return PathResult(false, f("No walking window found (recording %.1f s).\nKeep the phone still in a pocket for the walk; Algorithm 2.0 needs a stretch where gravity stays within %.0f deg of one direction.", t.last() - t[0], cfg.walkTiltDeg))

        // ---- accelerometer + magnetometer stages
        val fCad = strideFrequency(al.u, a, b, fs, fHi = 2.0, minSeconds = 8.0)
        val acc = al.acc?.let { accelAnalysis(t, it, fs, a, b, fCad) }
        val mag = al.mag?.let { magAnalysis(t, it, al.u, fs, a, b, turns) }
        val turnsM = if (mag != null) turns.mapIndexed { k, tr -> tr.copy(magAngle = mag.turnAngles[k]) } else turns

        // ---- gait cycles from gravity oscillation (Algorithm 2.0 step detector)
        val x = Dsp.bandpass(gravityPc(al.u, a, b), fs, 0.5, 1.5)
        val pk = Dsp.findPeaks(x, (0.9 * fs).toInt(), 0.3 * Dsp.std(x))
        val head = headingSeries(t, a, b, turnsM)

        // ---- stride length per gait cycle
        val stepsInWin = if (acc != null) acc.stepT.indices.filter { acc.stepT[it] >= t[a] && acc.stepT[it] <= t[b - 1] } else emptyList()
        val globalMean = if (stepsInWin.isNotEmpty()) stepsInWin.map { acc!!.stepLen[it] }.average() else Double.NaN
        val useAcc = opt.useAccelStride && acc != null && !globalMean.isNaN()
        val bear0 = Math.toRadians(opt.startBearingDeg)
        val east = DoubleArray(pk.size + 1); val north = DoubleArray(pk.size + 1); val times = DoubleArray(pk.size)
        var prevT = if (pk.isNotEmpty()) t[a + pk[0]] - 1.0 else 0.0
        var strideSum = 0.0
        for ((k, i) in pk.withIndex()) {
            val tp = t[a + i]
            val stride = if (useAcc) {
                val inCycle = stepsInWin.filter { acc!!.stepT[it] > prevT && acc.stepT[it] <= tp }
                val mean = if (inCycle.isNotEmpty()) inCycle.map { acc!!.stepLen[it] }.average() else globalMean
                (2 * mean).coerceIn(0.8, 2.0)
            } else opt.strideM
            strideSum += stride
            val beta = bear0 - head[i]                               // + heading = left = bearing decreases
            east[k + 1] = east[k] + stride * sin(beta); north[k + 1] = north[k] + stride * cos(beta)
            times[k] = tp; prevT = tp
        }
        fun pointAt(time: Double) = times.count { it <= time }.coerceIn(0, pk.size)

        val rejected = info.rejected.sortedBy { it.t0 }
        val turnPoint = IntArray(turnsM.size) { pointAt(0.5 * (turnsM[it].t0 + turnsM[it].t1)) }
        val anomalyPoint = IntArray(mag?.anomalies?.size ?: 0) { pointAt(0.5 * (mag!!.anomalies[it].tStart + mag.anomalies[it].tEnd)) }

        // ---- report
        val sb = StringBuilder()
        sb.append(f("Recording %.1f s | walking window %.1f-%.1f s (%.1f s)\n", t.last() - t[0], t[a] - t[0], t[b - 1] - t[0], t[b - 1] - t[a]))
        sb.append(f("Gait-cycle freq %s | sway cut-off %.2f Hz | yaw bias %+.2f deg/s%s\n",
            info.fStride?.let { f("%.2f Hz", it) } ?: "n/a", info.cutoff, Math.toDegrees(info.bias), if (al.gravityDerived) " | gravity derived from accel" else ""))
        sb.append(f("\nTURNS (Algorithm 2.0): %d accepted, %d rejected\n", turnsM.size, rejected.size))
        for ((k, tr) in turnsM.withIndex()) {
            val chk = if (tr.magAngle.isNaN()) "" else {
                val ok = abs(tr.angle - tr.magAngle) <= max(30.0, 0.25 * abs(tr.angle))
                f("  mag %+.0f deg %s", tr.magAngle, if (ok) "OK" else "MISMATCH")
            }
            sb.append(f(" #%d %5.1f-%5.1f s %+7.1f deg %s  peak %.2f rad/s  grav-move %.0f deg  conf %.2f%s%s\n", k + 1,
                tr.t0 - t[0], tr.t1 - t[0], tr.angle, if (tr.angle > 0) "LEFT " else "RIGHT", tr.peak, tr.gravityMove, tr.confidence,
                if (tr.touchesEdge) " [edge]" else "", chk))
        }
        for (tr in rejected) sb.append(f(" rejected %5.1f-%5.1f s %+6.1f deg (%s)\n", tr.t0 - t[0], tr.t1 - t[0], tr.angle, tr.reason))
        sb.append(f("Net heading change: %+.1f deg\n", turnsM.sumOf { it.angle }))

        sb.append(f("\nPATH: %d gait cycles | stride: %s | distance %.1f m | end point E %.1f m, N %.1f m (start bearing %.0f deg)\n",
            pk.size, if (useAcc) "accelerometer/Weinberg (mean %.2f m/step)".format(Locale.US, globalMean) else f("fixed %.2f m/cycle", opt.strideM),
            strideSum, east.last(), north.last(), opt.startBearingDeg))

        if (acc != null) {
            sb.append(f("\nACCELEROMETER: activity %s | %d steps in walking window | Weinberg distance %.1f m | clipped %d, Hampel %d%s\n",
                acc.label, stepsInWin.size, stepsInWin.sumOf { acc.stepLen[it] }, acc.clipped, acc.hampel, acc.note?.let { " | $it" } ?: ""))
            if (acc.windowsText.isNotEmpty()) sb.append(acc.windowsText)
            if (acc.label == "JOG" || acc.label == "SPRINT")
                sb.append(" CAVEAT: a loose phone adds energy in the step band; a JOG/SPRINT label may just be pocket rattle.\n")
        } else sb.append("\nACCELEROMETER: no data (stride falls back to fixed length)\n")

        if (mag != null) {
            sb.append(f("\nMAGNETOMETER: mean |B| %.1f uT (baseline %.1f) | hard-iron bias [%.1f, %.1f, %.1f] uT%s | Hampel %d+%d | ~180 deg switchbacks %d\n",
                mag.meanB, MAG_BASELINE_UT, mag.bias[0], mag.bias[1], mag.bias[2], if (mag.calibrated) "" else " (not applied: too little rotation)", mag.nOutXyz, mag.nOutB, mag.switchbacks))
            val dips = mag.anomalies.filter { !it.spike }.sumOf { it.tEnd - it.tStart }
            sb.append(f(" Over concrete/rebar (<%.0f uT): %.1f s | metal objects (>%.0f uT): %d\n", MAG_DIP_UT, dips, MAG_SPIKE_UT, mag.anomalies.count { it.spike }))
            for (e in mag.anomalies) sb.append(f("   [%5.1f-%5.1f s] %s extreme %.1f uT conf %.2f\n", e.tStart - t[0], e.tEnd - t[0], e.kind, e.extreme, e.confidence))
            sb.append(" Note: |B| thresholds are Vithura-local; elsewhere they can over-flag.\n")
        } else sb.append("\nMAGNETOMETER: no data (turn cross-check and anomaly map skipped)\n")

        return PathResult(true, sb.toString(), east, north, times, turnsM, rejected, turnPoint, mag?.anomalies ?: emptyList(), anomalyPoint)
    }
}
