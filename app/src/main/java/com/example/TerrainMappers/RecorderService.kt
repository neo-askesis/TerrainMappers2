package com.example.TerrainMappers

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.os.IBinder
import android.os.PowerManager
import android.os.SystemClock

/** Growable, thread-safe time series of (t seconds, x, y, z). */
class Buf {
    private var t = DoubleArray(1 shl 16)
    private var x = DoubleArray(1 shl 16)
    private var y = DoubleArray(1 shl 16)
    private var z = DoubleArray(1 shl 16)
    private var n = 0

    @Synchronized fun add(ts: Double, a: Double, b: Double, c: Double) {
        if (n == t.size) {
            val m = n * 2
            t = t.copyOf(m); x = x.copyOf(m); y = y.copyOf(m); z = z.copyOf(m)
        }
        t[n] = ts; x[n] = a; y[n] = b; z[n] = c; n++
    }
    @Synchronized fun size() = n
    @Synchronized fun clear() { n = 0 }
    @Synchronized fun snapshot(): RawSeries? =
        if (n == 0) null else RawSeries(t.copyOf(n), x.copyOf(n), y.copyOf(n), z.copyOf(n))
}

/** Process-wide recording state shared by the service (writer) and the activity (reader). */
object Session {
    val gyro = Buf()
    val grav = Buf()
    val accel = Buf()
    val mag = Buf()
    @Volatile var recording = false
    @Volatile var startedAtMs = 0L
    @Volatile var error: String? = null
    @Volatile var missing: String = ""

    fun reset() { gyro.clear(); grav.clear(); accel.clear(); mag.clear(); missing = "" }
    fun toRecording() = Recording(gyro.snapshot(), grav.snapshot(), accel.snapshot(), mag.snapshot())
}

/**
 * Foreground service that records Gyroscope, Gravity, Accelerometer and Magnetometer through the plain Android
 * SensorManager (the same API Phyphox itself uses). A foreground service + partial wake lock is what lets the
 * sensors keep delivering events while the screen is off and the phone is in your pocket.
 */
class RecorderService : Service(), SensorEventListener {
    private var thread: HandlerThread? = null
    private var wake: PowerManager.WakeLock? = null
    private var sm: SensorManager? = null

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        try {
            startInForeground()
        } catch (e: Exception) {
            Session.error = "Could not start the foreground service: ${e.message}"
            stopSelf()
            return START_NOT_STICKY
        }
        if (Session.recording) return START_NOT_STICKY
        Session.error = null
        Session.reset()

        val manager = getSystemService(SENSOR_SERVICE) as SensorManager
        sm = manager
        val th = HandlerThread("pocketpath-sensors").also { it.start() }
        thread = th
        val handler = Handler(th.looper)
        val missing = ArrayList<String>()
        fun reg(type: Int, name: String) {
            val s = manager.getDefaultSensor(type)
            if (s == null) missing.add(name) else manager.registerListener(this, s, 20_000, handler) // 50 Hz
        }
        reg(Sensor.TYPE_GYROSCOPE, "gyroscope")
        reg(Sensor.TYPE_GRAVITY, "gravity")
        reg(Sensor.TYPE_ACCELEROMETER, "accelerometer")
        reg(Sensor.TYPE_MAGNETIC_FIELD, "magnetometer")
        Session.missing = missing.joinToString()

        val pm = getSystemService(POWER_SERVICE) as PowerManager
        wake = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "pocketpath:recording").apply { acquire(4 * 60 * 60 * 1000L) }

        Session.startedAtMs = SystemClock.elapsedRealtime()
        Session.recording = true
        return START_NOT_STICKY
    }

    private fun startInForeground() {
        val channel = "pocketpath_rec"
        val nm = getSystemService(NotificationManager::class.java)
        nm.createNotificationChannel(NotificationChannel(channel, "Path recording", NotificationManager.IMPORTANCE_LOW))
        val pi = PendingIntent.getActivity(this, 0, Intent(this, MainActivity::class.java), PendingIntent.FLAG_IMMUTABLE)
        val n = Notification.Builder(this, channel)
            .setContentTitle("PocketPath is recording")
            .setContentText("Walk with the phone in your pocket, then open the app and press Stop.")
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setContentIntent(pi)
            .setOngoing(true)
            .build()
        if (Build.VERSION.SDK_INT >= 30) startForeground(1, n, ServiceInfo.FOREGROUND_SERVICE_TYPE_HEALTH)
        else startForeground(1, n)
    }

    override fun onSensorChanged(e: SensorEvent) {
        if (!Session.recording) return
        val t = e.timestamp * 1e-9                       // all sensors share this clock
        val a = e.values[0].toDouble(); val b = e.values[1].toDouble(); val c = e.values[2].toDouble()
        when (e.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> Session.gyro.add(t, a, b, c)          // rad/s
            Sensor.TYPE_GRAVITY -> Session.grav.add(t, a, b, c)            // m/s^2, points "up" like Phyphox
            Sensor.TYPE_ACCELEROMETER -> Session.accel.add(t, a, b, c)     // m/s^2 incl. gravity
            Sensor.TYPE_MAGNETIC_FIELD -> Session.mag.add(t, a, b, c)      // uT
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    override fun onDestroy() {
        Session.recording = false
        sm?.unregisterListener(this)
        thread?.quitSafely()
        wake?.let { if (it.isHeld) it.release() }
        super.onDestroy()
    }

    companion object {
        fun start(ctx: Context) {
            Session.error = null
            ctx.startForegroundService(Intent(ctx, RecorderService::class.java))
        }
        fun stop(ctx: Context) {
            Session.recording = false                    // stop accepting samples immediately
            ctx.stopService(Intent(ctx, RecorderService::class.java))
        }
    }
}
