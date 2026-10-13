package com.smartphonehmd.app

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Rebuilds H.264 frames from the PC's UDP video packets (format in the driver's video_protocol.h).
 * Calls [onFrame] for every complete frame and [onLoss] when a frame went missing.
 * Not thread-safe: use it from the receiving thread only.
 */
class FrameAssembler(
    private val onFrame: (data: ByteArray, size: Int, keyframe: Boolean, width: Int, height: Int) -> Unit,
    private val onLoss: () -> Unit,
) {
    private var buffer = ByteArray(512 * 1024)
    private var received = BooleanArray(0)
    private var currentFrame = -1L
    private var chunksReceived = 0
    private var chunkCount = 0
    private var frameSize = 0
    private var complete = true
    private var keyframe = false
    private var width = 0
    private var height = 0

    fun addPacket(packet: ByteArray, length: Int) {
        if (length < HEADER_SIZE || packet[0] != 'H'.code.toByte() || packet[1] != 'M'.code.toByte() ||
            packet[2] != 'V'.code.toByte() || packet[3] != '1'.code.toByte()
        ) {
            return
        }
        val header = ByteBuffer.wrap(packet, 0, HEADER_SIZE).order(ByteOrder.LITTLE_ENDIAN)
        val frameIndex = header.getInt(4).toLong() and 0xFFFFFFFFL
        val chunkIndex = header.getShort(8).toInt() and 0xFFFF
        val count = header.getShort(10).toInt() and 0xFFFF
        if (count == 0 || chunkIndex >= count) return

        if (frameIndex != currentFrame) {
            val behind = currentFrame - frameIndex
            if (currentFrame >= 0 && behind in 1..LATE_WINDOW) {
                return // a late packet from an older frame
            }
            // Anything else is a newer frame, or a new stream after the PC side restarted.
            val missedWholeFrames = currentFrame >= 0 && frameIndex != currentFrame + 1 && behind <= 0
            if ((currentFrame >= 0 && !complete) || missedWholeFrames) onLoss()
            startFrame(frameIndex, count, header)
        }
        if (complete || received[chunkIndex]) return

        val payload = length - HEADER_SIZE
        val offset = chunkIndex * MAX_PAYLOAD
        ensureCapacity(offset + payload)
        System.arraycopy(packet, HEADER_SIZE, buffer, offset, payload)
        received[chunkIndex] = true
        chunksReceived++
        if (chunkIndex == chunkCount - 1) frameSize = offset + payload

        if (chunksReceived == chunkCount) {
            complete = true
            onFrame(buffer, frameSize, keyframe, width, height)
        }
    }

    private fun startFrame(frameIndex: Long, count: Int, header: ByteBuffer) {
        currentFrame = frameIndex
        chunkCount = count
        chunksReceived = 0
        frameSize = 0
        complete = false
        keyframe = (header.get(12).toInt() and FLAG_KEYFRAME) != 0
        width = header.getShort(16).toInt() and 0xFFFF
        height = header.getShort(18).toInt() and 0xFFFF
        if (received.size < count) received = BooleanArray(count) else received.fill(false, 0, count)
    }

    private fun ensureCapacity(size: Int) {
        if (size > buffer.size) buffer = buffer.copyOf(maxOf(size, buffer.size * 2))
    }

    companion object {
        const val HEADER_SIZE = 20
        const val MAX_PAYLOAD = 1400
        const val FLAG_KEYFRAME = 1
        private const val LATE_WINDOW = 30L
    }
}
