package com.example.TerrainMappers

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.graphics.Typeface
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.text.InputType
import android.view.ViewGroup
import android.view.WindowInsets
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.Switch
import android.widget.TextView
import android.widget.Toast
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * PocketPath: records Gyroscope + Gravity + Accelerometer + Magnetometer, then reconstructs the walked path with
 * Algorithm 2.0 (gyro+gravity turns, gravity-oscillation gait cycles), accelerometer stride lengths and a
 * magnetometer turn cross-check. All UI is built in code, so no XML layouts are needed.
 */
class MainActivity : Activity() {
    private lateinit var strideEt: EditText
    private lateinit var bearingEt: EditText
    private lateinit var accSwitch: Switch
    private lateinit var startBtn: Button
    private lateinit var remapBtn: Button
    private lateinit var exportBtn: Button
    private lateinit var status: TextView
    private lateinit var report: TextView
    private lateinit var pathView: PathView

    private val ui = Handler(Looper.getMainLooper())
    private var lastRec: Recording? = null
    private var idleStatus = "Ready. Press Start, put the phone in your front pocket, walk, then press Stop."
    @Volatile private var analysing = false

    private val ticker = object : Runnable {
        override fun run() { refreshStatus(); ui.postDelayed(this, 500) }
    }

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    @Suppress("DEPRECATION")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setBackgroundColor(0xFFFFFFFF.toInt()) }
        root.setOnApplyWindowInsetsListener { v, ins ->
            val top: Int; val bottom: Int
            if (Build.VERSION.SDK_INT >= 30) {
                val i = ins.getInsets(WindowInsets.Type.systemBars()); top = i.top; bottom = i.bottom
            } else { top = ins.systemWindowInsetTop; bottom = ins.systemWindowInsetBottom }
            v.setPadding(dp(12), top + dp(8), dp(12), bottom + dp(8))
            ins
        }

        root.addView(TextView(this).apply { text = "PocketPath"; textSize = 20f; setTypeface(typeface, Typeface.BOLD) })

        strideEt = EditText(this).apply {
            inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL; setText("1.4"); setSingleLine()
        }
        bearingEt = EditText(this).apply {
            inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED
            setText("0"); setSingleLine()
        }
        val fields = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        fun field(title: String, et: EditText) = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            addView(TextView(context).apply { text = title; textSize = 11f })
            addView(et)
        }
        fields.addView(field("Fixed stride (m per gait cycle)", strideEt), LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        fields.addView(field("Start bearing (deg from north)", bearingEt), LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(fields)

        accSwitch = Switch(this).apply { text = "Accelerometer stride length (Weinberg)"; isChecked = true }
        root.addView(accSwitch)

        startBtn = Button(this).apply { text = "Start recording"; setOnClickListener { onStartStop() } }
        remapBtn = Button(this).apply { text = "Re-map"; setOnClickListener { remap() } }
        exportBtn = Button(this).apply { text = "Export CSV"; setOnClickListener { export() } }
        val buttons = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        buttons.addView(startBtn, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 2f))
        buttons.addView(remapBtn, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        buttons.addView(exportBtn, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(buttons)

        status = TextView(this).apply { textSize = 12f; setPadding(0, dp(4), 0, dp(4)) }
        root.addView(status)

        pathView = PathView(this)
        root.addView(pathView, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f))

        report = TextView(this).apply {
            textSize = 10.5f; typeface = Typeface.MONOSPACE; setTextIsSelectable(true); setPadding(dp(4), dp(4), dp(4), dp(4))
            text = "Report appears here."
        }
        val scroll = ScrollView(this).apply { addView(report) }
        root.addView(scroll, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, dp(190)))

        setContentView(root)
        root.requestApplyInsets()
    }

    override fun onResume() { super.onResume(); ui.post(ticker) }
    override fun onPause() { ui.removeCallbacks(ticker); super.onPause() }

    // ------------------------------------------------------------------ status
    private fun refreshStatus() {
        if (Session.recording) {
            val s = (SystemClock.elapsedRealtime() - Session.startedAtMs) / 1000
            status.text = String.format(Locale.US,
                "REC %02d:%02d   gyro %d | grav %d | accel %d | mag %d%s\nPut the phone in your pocket, walk, then come back and press Stop.",
                s / 60, s % 60, Session.gyro.size(), Session.grav.size(), Session.accel.size(), Session.mag.size(),
                if (Session.missing.isNotEmpty()) "\nNot available on this phone: ${Session.missing}" else "")
            startBtn.text = "Stop & map path"
        } else {
            startBtn.text = "Start recording"
            status.text = Session.error ?: idleStatus
        }
        startBtn.isEnabled = !analysing
    }

    // ------------------------------------------------------------------ start / stop
    private fun onStartStop() {
        if (Session.recording) stopAndAnalyse() else ensurePermissionsThenStart()
    }

    private fun ensurePermissionsThenStart() {
        val need = ArrayList<String>()
        if (Build.VERSION.SDK_INT >= 29 && checkSelfPermission(Manifest.permission.ACTIVITY_RECOGNITION) != PackageManager.PERMISSION_GRANTED)
            need.add(Manifest.permission.ACTIVITY_RECOGNITION)
        if (Build.VERSION.SDK_INT >= 33 && checkSelfPermission(Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED)
            need.add(Manifest.permission.POST_NOTIFICATIONS)
        if (need.isEmpty()) beginRecording() else requestPermissions(need.toTypedArray(), 42)
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != 42) return
        val arOk = Build.VERSION.SDK_INT < 29 || checkSelfPermission(Manifest.permission.ACTIVITY_RECOGNITION) == PackageManager.PERMISSION_GRANTED
        if (arOk) beginRecording()
        else Toast.makeText(this, "Physical-activity permission is required to keep the sensors running with the screen off.", Toast.LENGTH_LONG).show()
    }

    private fun beginRecording() {
        RecorderService.start(this)
        report.text = "Recording..."
        pathView.result = null
    }

    private fun options(): Options {
        val stride = strideEt.text.toString().toDoubleOrNull()?.coerceIn(0.3, 3.0) ?: 1.4
        val bearing = bearingEt.text.toString().toDoubleOrNull() ?: 0.0
        return Options(strideM = stride, useAccelStride = accSwitch.isChecked, startBearingDeg = bearing)
    }

    private fun stopAndAnalyse() {
        RecorderService.stop(this)
        val rec = Session.toRecording()
        lastRec = rec
        runAnalysis(rec)
    }

    private fun remap() {
        val rec = lastRec
        if (rec == null) Toast.makeText(this, "Nothing recorded yet.", Toast.LENGTH_SHORT).show()
        else if (Session.recording) Toast.makeText(this, "Stop the recording first.", Toast.LENGTH_SHORT).show()
        else runAnalysis(rec)
    }

    private fun runAnalysis(rec: Recording) {
        if (analysing) return
        analysing = true
        idleStatus = "Analysing..."
        refreshStatus()
        val opt = options()
        Thread {
            val res = try { PathEngine.analyze(rec, opt) } catch (e: Throwable) { PathResult(false, "Analysis failed: $e") }
            runOnUiThread {
                analysing = false
                pathView.result = res
                report.text = res.report
                idleStatus = if (res.ok) "Done. Change stride / bearing / stride model and press Re-map to recompute, or Export CSV."
                             else "Could not map this recording - see the report."
                refreshStatus()
            }
        }.start()
    }

    // ------------------------------------------------------------------ export (Phyphox-style CSVs: works with the Python scripts too)
    private fun export() {
        val rec = lastRec
        if (rec == null) { Toast.makeText(this, "Nothing recorded yet.", Toast.LENGTH_SHORT).show(); return }
        val res = pathView.result
        Thread {
            val msg = try {
                val dir = File(getExternalFilesDir(null), "recordings/" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()))
                dir.mkdirs()
                val all = listOfNotNull(rec.gyro, rec.grav, rec.accel, rec.mag)
                val t0 = all.minOfOrNull { it.t[0] } ?: 0.0
                fun write(name: String, label: String, s: RawSeries?) {
                    if (s == null) return
                    File(dir, name).bufferedWriter().use { o ->
                        o.write("\"Time (s)\",\"$label x\",\"$label y\",\"$label z\"\n")
                        for (i in s.t.indices) o.write(String.format(Locale.US, "%.6f,%.6f,%.6f,%.6f\n", s.t[i] - t0, s.x[i], s.y[i], s.z[i]))
                    }
                }
                write("Gyroscope.csv", "Gyroscope", rec.gyro)
                write("Gravity.csv", "Gravity", rec.grav)
                write("Accelerometer.csv", "Accelerometer", rec.accel)
                write("Magnetometer.csv", "Magnetometer", rec.mag)
                if (res != null) {
                    File(dir, "report.txt").writeText(res.report)
                    if (res.ok) File(dir, "path.csv").bufferedWriter().use { o ->
                        o.write("point,time_s,east_m,north_m\n")
                        for (i in res.east.indices)
                            o.write(String.format(Locale.US, "%d,%.3f,%.3f,%.3f\n", i, if (i == 0) 0.0 else res.cycleTimes[i - 1] - t0, res.east[i], res.north[i]))
                    }
                }
                "Saved to:\n${dir.absolutePath}"
            } catch (e: Exception) { "Export failed: ${e.message}" }
            runOnUiThread { Toast.makeText(this, msg, Toast.LENGTH_LONG).show(); idleStatus = msg; refreshStatus() }
        }.start()
    }
}
