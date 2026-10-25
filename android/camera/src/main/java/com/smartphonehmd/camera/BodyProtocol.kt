package com.smartphonehmd.camera

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * UDP packets between a body camera phone and the PC. Little-endian, with the same 20-byte header
 * as the headset's packets: magic "HMD1", type, 3 bytes of padding, seq (u32), timestamp (i64 ns).
 *
 * Video packet (type 10, phone to PC port 9946): a piece of one H.264 frame, 36 bytes + payload.
 * - seq: the frame's number, counting up by one per frame sent; timestamp: when the camera took
 *   it, on the phone's elapsedRealtimeNanos clock;
 * - 20 device id (u32), 24 picture width (u16), 26 picture height (u16), as the camera's sensor
 *   sees it; 28 piece index (u16), 30 piece count (u16); 32 quarter turns clockwise that make the
 *   picture upright (u8), 33 flags (u8: keyframe, exposure locked), 34 padding (u16);
 * - 36 the frame's bytes, [MAX_VIDEO_PAYLOAD] per piece except the last. Keyframes start with
 *   the stream's headers. The PC asks for one with a header-only keyframe request (type 11).
 *
 * Body packet (type 7, PC to phone): the pose found in one frame, 48 bytes + 20 per landmark:
 * - seq and timestamp: the video frame's;
 * - 20 device id (u32), 24 image width (u16), 26 image height (u16), both upright;
 * - 28 pose inference time in ms (f32), 32 flags (u8), 33 landmark count (u8), 34 padding (u16);
 * - 36 ball x, 40 ball y (0-1 across the upright image), 44 ball radius (over its width) (f32): a
 *   Move's bulb for calibration, with the ball flag, 0 when not seen;
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
    const val VIDEO_HEADER_SIZE = 36
    const val MAX_VIDEO_PAYLOAD = 1400

    const val TYPE_DISCOVER: Byte = 4
    const val TYPE_PC_HERE: Byte = 5
    const val TYPE_BODY: Byte = 7
    const val TYPE_TIME_PING: Byte = 8
    const val TYPE_TIME_PONG: Byte = 9
    const val TYPE_VIDEO: Byte = 10
    const val TYPE_KEYFRAME_REQUEST: Byte = 11

    const val FLAG_POSE = 1
    const val FLAG_EXPOSURE_LOCKED = 2
    const val FLAG_BALL = 4
    const val FLAG_KEYFRAME = 8

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

    /** Reads a body packet into [into]; false if [data] is not a whole body packet. */
    fun readBody(data: ByteArray, length: Int, into: Frame): Boolean {
        if (typeOf(data, length) != TYPE_BODY || length < BODY_HEADER_SIZE) return false
        val buffer = ByteBuffer.wrap(data, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        val count = data[33].toInt() and 0xFF
        if (count > LANDMARK_COUNT || length < BODY_HEADER_SIZE + count * LANDMARK_SIZE) return false
        into.seq = buffer.getInt(8)
        into.captureNs = buffer.getLong(12)
        into.deviceId = buffer.getInt(20)
        into.width = buffer.getShort(24).toInt() and 0xFFFF
        into.height = buffer.getShort(26).toInt() and 0xFFFF
        into.inferenceMs = buffer.getFloat(28)
        into.flags = data[32].toInt() and 0xFF
        into.ballX = buffer.getFloat(36)
        into.ballY = buffer.getFloat(40)
        into.ballRadius = buffer.getFloat(44)
        into.landmarkCount = count
        for (i in 0 until count * 5) into.landmarks[i] = buffer.getFloat(BODY_HEADER_SIZE + i * 4)
        return true
    }

    /**
     * Writes one piece of a video frame into [out] ([VIDEO_HEADER_SIZE] + [MAX_VIDEO_PAYLOAD]
     * bytes): [length] bytes of [data] from [offset]. Returns the packet's length.
     */
    fun writeVideo(
        out: ByteArray, seq: Int, captureNs: Long, deviceId: Int, width: Int, height: Int, piece: Int, pieces: Int,
        quarterTurns: Int, flags: Int, data: ByteArray, offset: Int, length: Int,
    ): Int {
        header(out, TYPE_VIDEO, seq, captureNs).apply {
            putInt(deviceId)
            putShort(width.toShort())
            putShort(height.toShort())
            putShort(piece.toShort())
            putShort(pieces.toShort())
            put(quarterTurns.toByte())
            put(flags.toByte())
            putShort(0)
            put(data, offset, length)
        }
        return VIDEO_HEADER_SIZE + length
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
