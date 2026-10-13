package com.smartphonehmd.app

import java.nio.ByteBuffer
import java.nio.ByteOrder
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class FrameAssemblerTest {
    private val frames = mutableListOf<Pair<ByteArray, Boolean>>()
    private var losses = 0
    private val assembler = FrameAssembler(
        onFrame = { data, size, keyframe, _, _ -> frames.add(data.copyOf(size) to keyframe) },
        onLoss = { losses++ },
    )

    /** Same packet layout the driver's video::Packetize produces. */
    private fun packets(frameIndex: Int, data: ByteArray, keyframe: Boolean = false): List<ByteArray> {
        val count = maxOf(1, (data.size + FrameAssembler.MAX_PAYLOAD - 1) / FrameAssembler.MAX_PAYLOAD)
        return (0 until count).map { chunk ->
            val start = chunk * FrameAssembler.MAX_PAYLOAD
            val end = minOf(data.size, start + FrameAssembler.MAX_PAYLOAD)
            ByteBuffer.allocate(FrameAssembler.HEADER_SIZE + end - start).order(ByteOrder.LITTLE_ENDIAN).apply {
                put("HMV1".toByteArray(Charsets.US_ASCII))
                putInt(frameIndex)
                putShort(chunk.toShort())
                putShort(count.toShort())
                put((if (keyframe) 1 else 0).toByte())
                put(ByteArray(3))
                putShort(1920)
                putShort(1080)
                put(data, start, end - start)
            }.array()
        }
    }

    private fun send(packet: ByteArray) = assembler.addPacket(packet, packet.size)

    private fun frameData(size: Int, seed: Int) = ByteArray(size) { (it * 31 + seed).toByte() }

    @Test
    fun rebuildsAFrameFromOutOfOrderChunks() {
        val data = frameData(3000, 1)
        packets(0, data, keyframe = true).reversed().forEach(::send)

        assertEquals(1, frames.size)
        assertArrayEquals(data, frames[0].first)
        assertTrue(frames[0].second)
        assertEquals(0, losses)
    }

    @Test
    fun reportsALossWhenAChunkIsMissing() {
        val first = packets(0, frameData(3000, 1))
        send(first[0])
        send(first[2]) // chunk 1 never arrives
        packets(1, frameData(500, 2)).forEach(::send)

        assertEquals(1, frames.size) // only frame 1
        assertEquals(1, losses)
    }

    @Test
    fun reportsALossWhenAWholeFrameIsMissing() {
        packets(0, frameData(100, 1)).forEach(::send)
        packets(2, frameData(100, 3)).forEach(::send)

        assertEquals(2, frames.size)
        assertEquals(1, losses)
    }

    @Test
    fun ignoresLatePackets() {
        packets(10, frameData(100, 1)).forEach(::send)
        packets(9, frameData(100, 2)).forEach(::send)

        assertEquals(1, frames.size)
        assertEquals(0, losses)
    }

    @Test
    fun acceptsAStreamThatRestartsFromZero() {
        packets(5000, frameData(100, 1), keyframe = true).forEach(::send)
        packets(0, frameData(100, 2), keyframe = true).forEach(::send)

        assertEquals(2, frames.size)
        assertArrayEquals(frameData(100, 2), frames[1].first)
    }
}
