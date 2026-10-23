package com.smartphonehmd.camera

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class BodyProtocolTest {
    // The bytes come from tools/body_tracker.py's pack_body and pack_video, so the phone and the
    // PC agree on the layout.
    @Test
    fun readsTheBodyPacketOfThePc() {
        val data = ("484d44310700000007000000141a99be1c00000004030201e001800200004841030200000000803e0000003f0000003e" +
            "cdcccc3dcdcc4c3e9a9999be6666663f3333733fcdcccc3e0000003f9a99193f3333333fcdcc4c3f").unhex()
        val frame = BodyProtocol.Frame()

        assertTrue(BodyProtocol.readBody(data, data.size, frame))

        assertEquals(7, frame.seq)
        assertEquals(123456789012L, frame.captureNs)
        assertEquals(0x01020304, frame.deviceId)
        assertEquals(480, frame.width)
        assertEquals(640, frame.height)
        assertEquals(12.5f, frame.inferenceMs)
        assertEquals(3, frame.flags)
        assertEquals(0.125f, frame.ballRadius)
        assertEquals(2, frame.landmarkCount)
        assertArrayEquals(
            floatArrayOf(0.1f, 0.2f, -0.3f, 0.9f, 0.95f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f),
            frame.landmarks.copyOf(10),
            0f,
        )
    }

    @Test
    fun rejectsCutBodyPackets() {
        val data = ("484d44310700000007000000141a99be1c00000004030201e001800200004841030200000000803e0000003f0000003e" +
            "cdcccc3dcdcc4c3e9a9999be6666663f3333733fcdcccc3e0000003f9a99193f3333333fcdcc4c3f").unhex()
        assertFalse(BodyProtocol.readBody(data, data.size - 1, BodyProtocol.Frame()))
        data[4] = BodyProtocol.TYPE_TIME_PONG
        assertFalse(BodyProtocol.readBody(data, data.size, BodyProtocol.Frame()))
    }

    @Test
    fun videoPacketMatchesThePcReader() {
        val out = ByteArray(BodyProtocol.VIDEO_HEADER_SIZE + BodyProtocol.MAX_VIDEO_PAYLOAD)
        val frame = byteArrayOf(9, 9, 0, 0, 0, 1, 0x67, 9)

        val length = BodyProtocol.writeVideo(
            out, seq = 5, captureNs = 123456789012L, deviceId = 0x01020304, width = 640, height = 480,
            piece = 0, pieces = 1, quarterTurns = 1, flags = 9, data = frame, offset = 2, length = 5,
        )

        assertEquals(41, length)
        assertEquals("484d44310a00000005000000141a99be1c000000040302018002e00100000100010900000000000167", out.copyOf(length).hex())
    }

    @Test
    fun pongMatchesThePcReader() {
        val out = ByteArray(BodyProtocol.PONG_SIZE)
        BodyProtocol.writePong(out, pingSeq = 42, phoneNs = 5_000_000_000L, pcNs = -7L, deviceId = 0x0A0B0C0D)
        assertEquals("484d4431090000002a00000000f2052a01000000f9ffffffffffffff0d0c0b0a", out.hex())
    }

    @Test
    fun readsTheHeaderOfOurPacketsOnly() {
        val ping = ByteArray(BodyProtocol.PONG_SIZE)
        BodyProtocol.writePong(ping, pingSeq = 42, phoneNs = 5_000_000_000L, pcNs = 0L, deviceId = 1)
        assertEquals(BodyProtocol.TYPE_TIME_PONG, BodyProtocol.typeOf(ping, ping.size))
        assertEquals(42, BodyProtocol.seqOf(ping))
        assertEquals(5_000_000_000L, BodyProtocol.timestampOf(ping))
        assertEquals(0.toByte(), BodyProtocol.typeOf(ping, BodyProtocol.HEADER_SIZE - 1))
        ping[0] = 'X'.code.toByte()
        assertEquals(0.toByte(), BodyProtocol.typeOf(ping, ping.size))
    }

    @Test
    fun discoverIsAHeaderOnly() {
        val out = ByteArray(BodyProtocol.HEADER_SIZE)
        BodyProtocol.writeDiscover(out)
        assertArrayEquals(
            byteArrayOf('H'.code.toByte(), 'M'.code.toByte(), 'D'.code.toByte(), '1'.code.toByte(), 4) + ByteArray(15),
            out,
        )
    }

    private fun ByteArray.hex() = joinToString("") { "%02x".format(it) }

    private fun String.unhex() = chunked(2).map { it.toInt(16).toByte() }.toByteArray()
}
