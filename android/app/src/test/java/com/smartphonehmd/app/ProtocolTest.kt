package com.smartphonehmd.app

import java.nio.ByteBuffer
import java.nio.ByteOrder
import org.junit.Assert.assertEquals
import org.junit.Test

class ProtocolTest {
    @Test
    fun sensorPacketMatchesTheDriverLayout() {
        val out = ByteArray(Protocol.SENSOR_PACKET_SIZE)
        Protocol.writeSensors(
            out,
            seq = 42,
            timestampNs = 123456789012345L,
            gyro = floatArrayOf(1.5f, -2f, 0.25f),
            accel = floatArrayOf(9.81f, 0f, 0f),
            mag = floatArrayOf(10f, 20f, 30f),
        )

        assertEquals("HMD1", String(out, 0, 4, Charsets.US_ASCII))
        val buffer = ByteBuffer.wrap(out).order(ByteOrder.LITTLE_ENDIAN)
        assertEquals(1, buffer.get(4).toInt())
        assertEquals(42, buffer.getInt(8))
        assertEquals(123456789012345L, buffer.getLong(12))
        assertEquals(1.5f, buffer.getFloat(20))
        assertEquals(-2f, buffer.getFloat(24))
        assertEquals(9.81f, buffer.getFloat(32))
        assertEquals(30f, buffer.getFloat(52))
    }

    @Test
    fun recenterPacketIsHeaderOnly() {
        val out = ByteArray(Protocol.SENSOR_PACKET_SIZE)
        Protocol.writeCommand(out, Protocol.TYPE_RECENTER, seq = 7, timestampNs = 99L)
        val buffer = ByteBuffer.wrap(out).order(ByteOrder.LITTLE_ENDIAN)
        assertEquals("HMD1", String(out, 0, 4, Charsets.US_ASCII))
        assertEquals(2, buffer.get(4).toInt())
        assertEquals(7, buffer.getInt(8))
        assertEquals(99L, buffer.getLong(12))
    }
}
