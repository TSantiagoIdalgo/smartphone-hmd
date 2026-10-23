package com.smartphonehmd.camera

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * UDP packets between a body camera phone and the PC. Little-endian, with the same 20-byte header
 * as the headset's packets: magic "HMD1", type, 3 bytes of padding, seq (u32), timestamp (i64 ns).
 *
 * Body packet (type 7, phone to PC port 9946), 48 bytes + 20 per landmark:
 * - timestamp: when the camera took the frame, on the phone's elapsedRealtimeNanos clock;
 * - 20 device id (u32), 24 image width (u16), 26 image height (u16), both upright;
 * - 28 pose inference time in ms (f32), 32 flags (u8), 33 landmark count (u8), 34 padding (u16);
 * - 36 ball x, 40 ball y, 44 ball radius (f32), a Move's bulb for calibration, 0 when not seen;
 * - 48 per landmark: x, y (0-1 across the upright image), z, visibility, presence (f32).
 *
 * Time sync: the PC sends a ping (type 8) with its own clock in the timestamp. The phone answers
 * at once with a pong (type 9): the ping's seq, the phone's clock in the timestamp, then the PC's
 * time echoed back (i64) and the device id (u32). The PC works out the clock offset from that.
 */
object BodyProtocol {
    const val BODY_PORT = 9946
    /** The headset driver answers discovery on its sensor port too, so either finds the PC. */
    const val DRIVER_PORT = 9944
    const val HEADER_SIZE = 20
    const val BODY_HEADER_SIZE = 48
    const val LANDMARK_SIZE = 20
    const val LANDMARK_COUNT = 33
    const val BODY_PACKET_SIZE = BODY_HEADER_SIZE + LANDMARK_SIZE * LANDMARK_COUNT
    const val PONG_SIZE = 32

    const val TYPE_DISCOVER: Byte = 4
    const val TYPE_PC_HERE: Byte = 5
    const val TYPE_BODY: Byte = 7
    const val TYPE_TIME_PING: Byte = 8
    const val TYPE_TIME_PONG: Byte = 9

    const val FLAG_POSE = 1
    const val FLAG_EXPOSURE_LOCKED = 2
    const val FLAG_BALL = 4

    private val MAGIC = byteArrayOf('H'.code.toByte(), 'M'.code.toByte(), 'D'.code.toByte(), '1'.code.toByte())

    /** One frame's result. [landmarks] holds 5 floats per landmark: x, y, z, visibility, presence. */
    class Frame(
        var seq: Int = 0,
        var captureNs: Long = 0,
        var deviceId: Int = 0,
        var width: Int = 0,
        var height: Int = 0,
        var inferenceMs: Float = 0f,
        var flags: Int = 0,
        var ballX: Float = 0f,
        var ballY: Float = 0f,
        var ballRadius: Float = 0f,
        var landmarkCount: Int = 0,
        val landmarks: FloatArray = FloatArray(LANDMARK_COUNT * 5),
    )

    /** Writes [frame] into [out] ([BODY_PACKET_SIZE] bytes) and returns the packet's length. */
    fun writeBody(out: ByteArray, frame: Frame): Int {
        val count = frame.landmarkCount.coerceIn(0, LANDMARK_COUNT)
        val buffer = header(out, TYPE_BODY, frame.seq, frame.captureNs)
        buffer.putInt(frame.deviceId)
        buffer.putShort(frame.width.toShort())
        buffer.putShort(frame.height.toShort())
        buffer.putFloat(frame.inferenceMs)
        buffer.put(frame.flags.toByte())
        buffer.put(count.toByte())
        buffer.putShort(0)
        buffer.putFloat(frame.ballX)
        buffer.putFloat(frame.ballY)
        buffer.putFloat(frame.ballRadius)
        for (i in 0 until count * 5) buffer.putFloat(frame.landmarks[i])
        return BODY_HEADER_SIZE + count * LANDMARK_SIZE
    }

    /** Writes a header-only discover packet into [out] ([HEADER_SIZE] bytes). */
    fun writeDiscover(out: ByteArray) {
        header(out, TYPE_DISCOVER, 0, 0)
    }

    /** Writes the answer to a ping into [out] ([PONG_SIZE] bytes). */
    fun writePong(out: ByteArray, pingSeq: Int, phoneNs: Long, pcNs: Long, deviceId: Int) {
        header(out, TYPE_TIME_PONG, pingSeq, phoneNs).putLong(pcNs).putInt(deviceId)
    }

    /** The packet's type, or 0 if it is not one of ours. */
    fun typeOf(data: ByteArray, length: Int): Byte {
        if (length < HEADER_SIZE) return 0
        for (i in 0 until 4) if (data[i] != MAGIC[i]) return 0
        return data[4]
    }

    fun seqOf(data: ByteArray): Int = ByteBuffer.wrap(data, 8, 4).order(ByteOrder.LITTLE_ENDIAN).int

    fun timestampOf(data: ByteArray): Long = ByteBuffer.wrap(data, 12, 8).order(ByteOrder.LITTLE_ENDIAN).long

    private fun header(out: ByteArray, type: Byte, seq: Int, timestampNs: Long): ByteBuffer =
        ByteBuffer.wrap(out).order(ByteOrder.LITTLE_ENDIAN).apply {
            put(MAGIC)
            put(type)
            put(ByteArray(3))
            putInt(seq)
            putLong(timestampNs)
        }
}
