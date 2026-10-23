package com.smartphonehmd.camera

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

class BodyProtocolTest {
    // The expected bytes come from tools/body_receiver.py's pack_body and PONG, so the phone and
    // the PC's reader agree on the layout.
    @Test
    fun bodyPacketMatchesThePcReader() {
        val frame = BodyProtocol.Frame(
            seq = 7, captureNs = 123456789012L, deviceId = 0x01020304, width = 480, height = 640,
            inferenceMs = 12.5f, flags = 3, ballX = 0.25f, ballY = 0.5f, ballRadius = 0.125f, landmarkCount = 2,
        )
        floatArrayOf(0.1f, 0.2f, -0.3f, 0.9f, 0.95f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f).copyInto(frame.landmarks)
        val out = ByteArray(BodyProtocol.BODY_PACKET_SIZE)

        val length = BodyProtocol.writeBody(out, frame)

        assertEquals(88, length)
        assertEquals(
            "484d44310700000007000000141a99be1c00000004030201e001800200004841030200000000803e0000003f0000003e" +
                "cdcccc3dcdcc4c3e9a9999be6666663f3333733fcdcccc3e0000003f9a99193f3333333fcdcc4c3f",
            out.copyOf(length).hex(),
        )
    }

    @Test
    fun fullPacketHoldsAllThePoints() {
        val frame = BodyProtocol.Frame(landmarkCount = 40)
        assertEquals(708, BodyProtocol.BODY_PACKET_SIZE)
        assertEquals(708, BodyProtocol.writeBody(ByteArray(BodyProtocol.BODY_PACKET_SIZE), frame))
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
}
