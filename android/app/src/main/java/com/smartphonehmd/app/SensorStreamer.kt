package com.smartphonehmd.app

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.concurrent.atomic.AtomicInteger

/**
 * Sends the raw gyroscope, accelerometer and magnetometer values to the PC over UDP.
 * One packet per gyroscope sample, carrying the latest accelerometer and magnetometer values; each
 * gyroscope sample also goes to [timewarp], with its packet's seq.
 * Once a second it also reports the latency [meter] measured, so the PC can predict the head that
 * far ahead (unless [predict] is off). Sensor callbacks and sends happen on a background thread.
 */
class SensorStreamer(
    private val sensorManager: SensorManager,
    private val meter: LatencyMeter,
    private val timewarp: Timewarp,
) : SensorEventListener {
    private var thread: HandlerThread? = null
    @Volatile private var handler: Handler? = null
    private var socket: DatagramSocket? = null
    @Volatile private var target: InetAddress? = null

    private val accel = FloatArray(3)
    private val mag = FloatArray(3)
    private val buffer = ByteArray(Protocol.SENSOR_PACKET_SIZE)
    private var seq = 0
    private var nextLatencyReportNs = 0L

    val packetsSent = AtomicInteger(0)
    @Volatile private var setupError: String? = null
    @Volatile private var sendError: String? = null
    val lastError: String? get() = setupError ?: sendError
    @Volatile var predict = true

    /** The PC's address, or null while there is none yet. */
    val targetHost: String? get() = target?.hostAddress

    /** Starts sending to [host]; with no host, sensors only start flowing once [setTarget] gives one. */
    fun start(host: String?) {
        stop()
        setupError = null
        sendError = null
        packetsSent.set(0)
        target = null
        val newThread = HandlerThread("sensor-streamer").apply { start() }
        val newHandler = Handler(newThread.looper)
        thread = newThread
        handler = newHandler

        // An exception escaping this thread would close the app, so failures become lastError.
        newHandler.post {
            if (!host.isNullOrEmpty()) {
                try {
                    target = InetAddress.getByName(host)
                } catch (e: Exception) {
                    sendError = e.message ?: e.toString() // the PC finder may still provide one
                }
            }
            try {
                socket = DatagramSocket()
                for (type in intArrayOf(Sensor.TYPE_GYROSCOPE, Sensor.TYPE_ACCELEROMETER, Sensor.TYPE_MAGNETIC_FIELD)) {
                    val sensor = sensorManager.getDefaultSensor(type)
                    if (sensor == null) {
                        if (type == Sensor.TYPE_GYROSCOPE) setupError = "Este móvil no tiene giroscopio"
                        continue
                    }
                    sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_FASTEST, newHandler)
                }
            } catch (e: Exception) {
                setupError = e.message ?: e.toString()
            }
        }
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        val oldThread = thread ?: return
        handler?.post {
            socket?.close()
            socket = null
        }
        oldThread.quitSafely()
        thread = null
        handler = null
    }

    /** Sends to [host] from now on, for example when the PC changed address. */
    fun setTarget(host: String) {
        handler?.post {
            try {
                target = InetAddress.getByName(host)
            } catch (e: Exception) {
                sendError = e.message ?: e.toString()
            }
        }
    }

    fun recenter() = sendCommand(Protocol.TYPE_RECENTER)

    /** Asks the PC for a video keyframe. Safe to call from any thread. */
    fun requestKeyframe() = sendCommand(Protocol.TYPE_KEYFRAME_REQUEST)

    private fun sendCommand(type: Byte) {
        handler?.post {
            Protocol.writeCommand(buffer, type, ++seq, SystemClock.elapsedRealtimeNanos())
            send(Protocol.HEADER_SIZE)
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> event.values.copyInto(accel, endIndex = 3)
            Sensor.TYPE_MAGNETIC_FIELD -> event.values.copyInto(mag, endIndex = 3)
            Sensor.TYPE_GYROSCOPE -> {
                Protocol.writeSensors(buffer, ++seq, event.timestamp, event.values, accel, mag)
                timewarp.onGyro(seq, event.timestamp, event.values)
                val now = SystemClock.elapsedRealtimeNanos()
                if (send(Protocol.SENSOR_PACKET_SIZE)) meter.sent(seq, now)
                if (now >= nextLatencyReportNs) {
                    nextLatencyReportNs = now + LATENCY_REPORT_INTERVAL_NS
                    val latency = if (predict) meter.averageMs() ?: 0f else 0f
                    Protocol.writeLatency(buffer, ++seq, now, latency)
                    send(Protocol.LATENCY_PACKET_SIZE)
                }
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    // Errors (the network dropping out, for one) only show on screen: sending carries on and
    // recovers by itself when the network comes back.
    private fun send(length: Int): Boolean {
        val s = socket ?: return false
        val t = target ?: return false
        return try {
            s.send(DatagramPacket(buffer, length, t, Protocol.SENSOR_PORT))
            packetsSent.incrementAndGet()
            if (sendError != null) sendError = null
            true
        } catch (e: Exception) {
            sendError = e.message ?: e.toString()
            false
        }
    }

    private companion object {
        const val LATENCY_REPORT_INTERVAL_NS = 1_000_000_000L
    }
}
