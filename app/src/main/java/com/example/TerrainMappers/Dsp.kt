package com.example.TerrainMappers

import kotlin.math.*

/**
 * Small dependency-free DSP toolbox mirroring the SciPy / NumPy calls used by the Python prototypes
 * (butter + filtfilt, Hampel, rfft power spectrum, find_peaks, interp).
 */
object Dsp {

    // ------------------------------------------------------------------ Butterworth (biquad cascade)
    class Biquad(val b: DoubleArray, val a: DoubleArray)           // a[0] == 1

    private fun sections(order: Int, fc: Double, fs: Double, high: Boolean): List<Biquad> {
        require(order % 2 == 0) { "even orders only" }
        val w0 = 2.0 * PI * min(fc, 0.49 * fs) / fs
        val cw = cos(w0)
        val sw = sin(w0)
        return List(order / 2) { k ->
            val q = 1.0 / (2.0 * sin((2 * k + 1) * PI / (2.0 * order)))   // Butterworth pole-pair Q
            val alpha = sw / (2.0 * q)
            val a0 = 1.0 + alpha
            val b = if (!high) doubleArrayOf((1 - cw) / 2, 1 - cw, (1 - cw) / 2)
                    else doubleArrayOf((1 + cw) / 2, -(1 + cw), (1 + cw) / 2)
            Biquad(DoubleArray(3) { b[it] / a0 }, doubleArrayOf(1.0, -2.0 * cw / a0, (1.0 - alpha) / a0))
        }
    }

    /** Forward IIR with steady-state initial conditions (like scipy lfilter with zi = lfilter_zi * x[0]). */
    private fun lfilter(sos: List<Biquad>, x: DoubleArray): DoubleArray {
        var cur = x
        for (s in sos) {
            val gain = s.b.sum() / s.a.sum()
            val u0 = cur[0]
            val y0 = u0 * gain
            var z2 = s.b[2] * u0 - s.a[2] * y0
            var z1 = s.b[1] * u0 - s.a[1] * y0 + z2
            val y = DoubleArray(cur.size)
            for (i in cur.indices) {
                val xi = cur[i]
                val yi = s.b[0] * xi + z1
                z1 = s.b[1] * xi - s.a[1] * yi + z2
                z2 = s.b[2] * xi - s.a[2] * yi
                y[i] = yi
            }
            cur = y
        }
        return cur
    }

    /** Zero-phase forward-backward filtering with odd edge extension (same idea as scipy.signal.filtfilt). */
    fun filtfilt(sos: List<Biquad>, x: DoubleArray): DoubleArray {
        val n = x.size
        if (n < 4) return x.copyOf()
        val pad = min(3 * (2 * sos.size + 1), n - 1)
        val ext = DoubleArray(n + 2 * pad)
        for (i in 0 until pad) ext[i] = 2 * x[0] - x[pad - i]
        for (i in 0 until n) ext[pad + i] = x[i]
        for (i in 0 until pad) ext[pad + n + i] = 2 * x[n - 1] - x[n - 2 - i]
        val f = lfilter(sos, ext); f.reverse()
        val g = lfilter(sos, f); g.reverse()
        return g.copyOfRange(pad, pad + n)
    }

    fun lowpass(x: DoubleArray, fs: Double, fc: Double, order: Int = 2) = filtfilt(sections(order, fc, fs, false), x)
    fun highpass(x: DoubleArray, fs: Double, fc: Double, order: Int = 2) = filtfilt(sections(order, fc, fs, true), x)
    /** Band-pass = high-pass(lo) cascaded with low-pass(hi), each of the given order. */
    fun bandpass(x: DoubleArray, fs: Double, lo: Double, hi: Double, order: Int = 2) =
        filtfilt(sections(order, lo, fs, true) + sections(order, hi, fs, false), x)

    // ------------------------------------------------------------------ statistics
    fun median(v: DoubleArray): Double {
        if (v.isEmpty()) return Double.NaN
        val s = v.sortedArray(); val n = s.size
        return if (n % 2 == 1) s[n / 2] else 0.5 * (s[n / 2 - 1] + s[n / 2])
    }

    fun std(v: DoubleArray): Double {                                   // population std (numpy default)
        if (v.isEmpty()) return 0.0
        val m = v.average()
        return sqrt(v.sumOf { (it - m) * (it - m) } / v.size)
    }

    private fun rollingMedian(x: DoubleArray, window: Int): DoubleArray {
        val h = window / 2; val n = x.size
        val out = DoubleArray(n); val buf = DoubleArray(window + 1)
        for (i in 0 until n) {
            val lo = max(0, i - h); val hi = min(n - 1, i + h); val m = hi - lo + 1
            for (j in 0 until m) buf[j] = x[lo + j]
            java.util.Arrays.sort(buf, 0, m)
            out[i] = if (m % 2 == 1) buf[m / 2] else 0.5 * (buf[m / 2 - 1] + buf[m / 2])
        }
        return out
    }

    /** Hampel filter (identical maths to the Python scripts). Returns (cleaned, number of outliers). */
    fun hampel(x: DoubleArray, window: Int = 7, nSigma: Double = 3.0): Pair<DoubleArray, Int> {
        val med = rollingMedian(x, window)
        val diff = DoubleArray(x.size) { abs(x[it] - med[it]) }
        val mad = rollingMedian(diff, window)
        val out = x.copyOf(); var cnt = 0
        for (i in x.indices) if (diff[i] > nSigma * 1.4826 * mad[i]) { out[i] = med[i]; cnt++ }
        return out to cnt
    }

    // ------------------------------------------------------------------ spectrum
    fun nextPow2(n: Int): Int { var p = 1; while (p < n) p = p shl 1; return p }

    fun powerSpectrum(x: DoubleArray, nfft: Int): DoubleArray {
        val re = DoubleArray(nfft); val im = DoubleArray(nfft)
        System.arraycopy(x, 0, re, 0, min(x.size, nfft))
        fft(re, im)
        return DoubleArray(nfft / 2 + 1) { re[it] * re[it] + im[it] * im[it] }
    }

    private fun fft(re: DoubleArray, im: DoubleArray) {
        val n = re.size
        var j = 0
        for (i in 1 until n) {
            var bit = n shr 1
            while (j and bit != 0) { j = j xor bit; bit = bit shr 1 }
            j = j xor bit
            if (i < j) { var t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t }
        }
        var len = 2
        while (len <= n) {
            val ang = -2.0 * PI / len
            val wr = cos(ang); val wi = sin(ang)
            var i = 0
            while (i < n) {
                var cr = 1.0; var ci = 0.0
                for (k in 0 until len / 2) {
                    val ur = re[i + k]; val ui = im[i + k]
                    val vr = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci
                    val vi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr
                    re[i + k] = ur + vr; im[i + k] = ui + vi
                    re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi
                    val nr = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = nr
                }
                i += len
            }
            len = len shl 1
        }
    }

    fun hann(n: Int) = DoubleArray(n) { if (n == 1) 1.0 else 0.5 - 0.5 * cos(2.0 * PI * it / (n - 1)) }

    // ------------------------------------------------------------------ find_peaks (distance, then prominence - same order as SciPy)
    fun findPeaks(x: DoubleArray, distance: Int = 1, prominence: Double = 0.0): IntArray {
        val n = x.size
        if (n < 3) return IntArray(0)
        var peaks = ArrayList<Int>()
        var i = 1
        val iMax = n - 1
        while (i < iMax) {
            if (x[i - 1] < x[i]) {
                var ahead = i + 1
                while (ahead < iMax && x[ahead] == x[i]) ahead++
                if (x[ahead] < x[i]) { peaks.add((i + ahead - 1) / 2); i = ahead }
            }
            i++
        }
        if (distance > 1 && peaks.size > 1) {
            val keep = BooleanArray(peaks.size) { true }
            val order = peaks.indices.sortedBy { x[peaks[it]] }          // ascending height (stable)
            for (oi in order.indices.reversed()) {
                val jj = order[oi]
                if (!keep[jj]) continue
                var k = jj - 1
                while (k >= 0 && peaks[jj] - peaks[k] < distance) { keep[k] = false; k-- }
                k = jj + 1
                while (k < peaks.size && peaks[k] - peaks[jj] < distance) { keep[k] = false; k++ }
            }
            peaks = ArrayList(peaks.filterIndexed { idx, _ -> keep[idx] })
        }
        if (prominence > 0.0) peaks = ArrayList(peaks.filter { prominenceOf(x, it) >= prominence })
        return peaks.toIntArray()
    }

    private fun prominenceOf(x: DoubleArray, p: Int): Double {
        var lMin = x[p]; var k = p
        while (k >= 0 && x[k] <= x[p]) { lMin = min(lMin, x[k]); k-- }
        var rMin = x[p]; k = p
        while (k < x.size && x[k] <= x[p]) { rMin = min(rMin, x[k]); k++ }
        return x[p] - max(lMin, rMin)
    }

    // ------------------------------------------------------------------ misc
    /** np.interp: linear interpolation, clamped at both ends. tx must be strictly increasing. */
    fun interp(tq: DoubleArray, tx: DoubleArray, y: DoubleArray): DoubleArray {
        val out = DoubleArray(tq.size)
        var j = 0
        for (i in tq.indices) {
            val q = tq[i]
            if (q <= tx[0]) { out[i] = y[0]; continue }
            if (q >= tx[tx.size - 1]) { out[i] = y[y.size - 1]; continue }
            while (j < tx.size - 2 && tx[j + 1] < q) j++
            val f = (q - tx[j]) / (tx[j + 1] - tx[j])
            out[i] = y[j] + f * (y[j + 1] - y[j])
        }
        return out
    }

    /** np.searchsorted(side="left") on a sorted array. */
    fun searchSorted(a: DoubleArray, v: Double): Int {
        var lo = 0; var hi = a.size
        while (lo < hi) { val mid = (lo + hi) ushr 1; if (a[mid] < v) lo = mid + 1 else hi = mid }
        return lo
    }
}
