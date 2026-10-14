package com.smartphonehmd.app

import java.nio.ByteBuffer
import java.nio.ByteOrder

/** UDP packet format shared with the SteamVR driver (see docs/PLAN.md). Little-endian. */
object Protocol {
    const val SENSOR_PORT = 9944
    const val HEADER_SIZE = 20
    const val SENSOR_PACKET_SIZE = 56

    const val TYPE_SENSORS: Byte = 1
    const val TYPE_RECENTER: Byte = 2
    const val TYPE_KEYFRAME_REQUEST: Byte = 3

    private val MAGIC = byteArrayOf('H'.code.toByte(), 'M'.code.toByte(), 'D'.code.toByte(), '1'.code.toByte())

    /** Writes a sensor packet into [out], which must hold [SENSOR_PACKET_SIZE] bytes. Uses the first 3 values of each array. */
    fun writeSensors(
        out: ByteArray,
        seq: Int,
        timestampNs: Long,
        gyro: FloatArray,
        accel: FloatArray,
        mag: FloatArray,
    ) {
        val buffer = header(out, TYPE_SENSORS, seq, timestampNs)
        for (values in arrayOf(gyro, accel, mag)) {
            for (i in 0 until 3) buffer.putFloat(values[i])
        }
    }

    /** Writes a header-only command packet: recenter or keyframe request. */
    fun writeCommand(out: ByteArray, type: Byte, seq: Int, timestampNs: Long) {
        header(out, type, seq, timestampNs)
    }

    private fun header(out: ByteArray, type: Byte, seq: Int, timestampNs: Long): ByteBuffer =
        ByteBuffer.wrap(out).order(ByteOrder.LITTLE_ENDIAN).apply {
            put(MAGIC)
            put(type)
            put(ByteArray(3))
            putInt(seq)
            putLong(timestampNs)
        }
}
