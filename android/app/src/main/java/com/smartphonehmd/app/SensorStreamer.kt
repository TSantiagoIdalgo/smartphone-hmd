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
 * One packet per gyroscope sample, carrying the latest accelerometer and magnetometer values.
 * Sensor callbacks and sends happen on a background thread.
 */
class SensorStreamer(private val sensorManager: SensorManager) : SensorEventListener {
    private var thread: HandlerThread? = null
    @Volatile private var handler: Handler? = null
    private var socket: DatagramSocket? = null
    private var target: InetAddress? = null

    private val accel = FloatArray(3)
    private val mag = FloatArray(3)
    private val buffer = ByteArray(Protocol.SENSOR_PACKET_SIZE)
    private var seq = 0

    val packetsSent = AtomicInteger(0)
    @Volatile var lastError: String? = null
        private set

    fun start(host: String) {
        stop()
        lastError = null
        packetsSent.set(0)
        val newThread = HandlerThread("sensor-streamer").apply { start() }
        val newHandler = Handler(newThread.looper)
        thread = newThread
        handler = newHandler

        // An exception escaping this thread would close the app, so failures become lastError.
        newHandler.post {
            try {
                target = InetAddress.getByName(host)
                socket = DatagramSocket()
                for (type in intArrayOf(Sensor.TYPE_GYROSCOPE, Sensor.TYPE_ACCELEROMETER, Sensor.TYPE_MAGNETIC_FIELD)) {
                    val sensor = sensorManager.getDefaultSensor(type)
                    if (sensor == null) {
                        if (type == Sensor.TYPE_GYROSCOPE) lastError = "Este móvil no tiene giroscopio"
                        continue
                    }
                    sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_FASTEST, newHandler)
                }
            } catch (e: Exception) {
                lastError = e.message ?: e.toString()
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
                send(Protocol.SENSOR_PACKET_SIZE)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    private fun send(length: Int) {
        val s = socket ?: return
        try {
            s.send(DatagramPacket(buffer, length, target, Protocol.SENSOR_PORT))
            packetsSent.incrementAndGet()
        } catch (e: Exception) {
            lastError = e.message ?: e.toString()
        }
    }
}
