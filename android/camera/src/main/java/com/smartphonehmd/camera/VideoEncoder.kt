package com.smartphonehmd.camera

import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.os.Build
import android.os.Bundle
import java.nio.ByteBuffer
import java.util.ArrayDeque

/**
 * The phone's H.264 encoder, fed the camera's YUV frames. Frames go in from one thread with
 * [encode]; each encoded frame comes out on the encoder's own thread through [output], with the
 * stream's headers in front of every keyframe so the PC can start decoding at any of them.
 */
class VideoEncoder(
    val width: Int,
    val height: Int,
    val fps: Int,
    bitrate: Int,
    private val output: Output,
) {
    /** What the PC needs to know about a frame besides its picture. */
    class Tag(val captureNs: Long, val quarterTurns: Int, val flags: Int)

    fun interface Output {
        /** [data] from [offset] holds the frame; it is reused once this returns. */
        fun onFrame(tag: Tag, keyframe: Boolean, data: ByteArray, offset: Int, length: Int)
    }

    private val codec = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC)
    val name: String = codec.name

    // Tags of the frames inside the encoder, by their time in µs, oldest first.
    private val pending = ArrayDeque<Pair<Long, Tag>>()
    private var lastPtsUs = Long.MIN_VALUE
    @Volatile private var running = true
    private val drainThread: Thread

    // Encoder thread only.
    private var headers = ByteArray(0)
    private var out = ByteArray(64 * 1024)

    init {
        try {
            try {
                codec.configure(format(bitrate, lowLatency = true), null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
            } catch (_: Exception) {
                // An encoder that refuses one of the hints still takes the plain format.
                codec.reset()
                codec.configure(format(bitrate, lowLatency = false), null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
            }
            codec.start()
        } catch (e: Exception) {
            codec.release()
            throw e
        }
        drainThread = Thread({ drain() }, "video-encoder").apply { start() }
    }

    private fun format(bitrate: Int, lowLatency: Boolean) =
        MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, width, height).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible)
            setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
            setInteger(MediaFormat.KEY_FRAME_RATE, fps)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, KEYFRAME_INTERVAL_S)
            if (lowLatency) {
                setInteger(MediaFormat.KEY_PRIORITY, 0) // real time
                if (Build.VERSION.SDK_INT >= 29) setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0)
                if (Build.VERSION.SDK_INT >= 30) setInteger(MediaFormat.KEY_LATENCY, 1)
            }
        }

    /**
     * Hands one frame (planes Y, U, V of a [width] x [height] picture) to the encoder. False if
     * it had no room for it, so the frame is skipped. Always called from the same thread.
     */
    fun encode(planes: Array<YuvPlane>, tag: Tag): Boolean {
        val index = codec.dequeueInputBuffer(INPUT_WAIT_US)
        if (index < 0) return false
        // The encoder wants strictly increasing times.
        val ptsUs = maxOf(tag.captureNs / 1000, lastPtsUs + 1)
        lastPtsUs = ptsUs
        val image = codec.getInputImage(index)
        if (image != null) {
            val target = Array(3) { YuvPlane(image.planes[it].buffer, image.planes[it].rowStride, image.planes[it].pixelStride) }
            Yuv.copy(planes, target, width, height)
        } else {
            val semiPlanar = codec.inputFormat.getInteger(MediaFormat.KEY_COLOR_FORMAT) ==
                MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar
            Yuv.copy(planes, Yuv.layout(codec.getInputBuffer(index)!!, width, height, semiPlanar), width, height)
        }
        synchronized(pending) { pending.addLast(ptsUs to tag) }
        codec.queueInputBuffer(index, 0, width * height * 3 / 2, ptsUs, 0)
        return true
    }

    /** Makes the next frame a keyframe, which the PC can decode on its own. Any thread. */
    fun requestKeyframe() {
        try {
            codec.setParameters(Bundle().apply { putInt(MediaCodec.PARAMETER_KEY_REQUEST_SYNC_FRAME, 0) })
        } catch (_: IllegalStateException) {
            // Closing.
        }
    }

    /** Stops the encoder. Call from the thread that calls [encode]. */
    fun close() {
        running = false
        drainThread.join(1000)
        try {
            codec.stop()
        } catch (_: IllegalStateException) {
        }
        codec.release()
    }

    private fun drain() {
        val info = MediaCodec.BufferInfo()
        while (running) {
            val index = try {
                codec.dequeueOutputBuffer(info, OUTPUT_WAIT_US)
            } catch (_: IllegalStateException) {
                return
            }
            if (index < 0) continue
            try {
                val buffer = codec.getOutputBuffer(index)
                if (buffer != null && info.size > 0) {
                    buffer.position(info.offset)
                    buffer.limit(info.offset + info.size)
                    if (info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG != 0) {
                        headers = ByteArray(info.size).also { buffer.get(it) }
                    } else {
                        sendFrame(buffer, info)
                    }
                }
            } finally {
                codec.releaseOutputBuffer(index, false)
            }
        }
    }

    private fun sendFrame(buffer: ByteBuffer, info: MediaCodec.BufferInfo) {
        val tag = synchronized(pending) {
            // Frames the encoder dropped leave their tags behind; skip past them.
            var found: Tag? = null
            while (pending.isNotEmpty() && pending.peekFirst()!!.first <= info.presentationTimeUs) {
                val (pts, t) = pending.removeFirst()
                if (pts == info.presentationTimeUs) found = t
            }
            found
        } ?: return
        val keyframe = info.flags and MediaCodec.BUFFER_FLAG_KEY_FRAME != 0
        val start = headers.size
        if (out.size < start + info.size) out = ByteArray(start + info.size + 16 * 1024)
        buffer.get(out, start, info.size)
        // Most encoders give the headers once, at the start; some put them in each keyframe.
        if (keyframe && !H264.hasHeaders(out, start, info.size)) {
            headers.copyInto(out, 0)
            output.onFrame(tag, true, out, 0, start + info.size)
        } else {
            output.onFrame(tag, keyframe, out, start, info.size)
        }
    }

    private companion object {
        const val KEYFRAME_INTERVAL_S = 2
        const val INPUT_WAIT_US = 5_000L
        const val OUTPUT_WAIT_US = 10_000L
    }
}
