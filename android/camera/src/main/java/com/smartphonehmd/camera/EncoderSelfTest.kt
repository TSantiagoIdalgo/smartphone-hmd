package com.smartphonehmd.camera

import android.content.Context
import android.os.SystemClock
import android.util.Log
import java.io.ByteArrayOutputStream
import java.io.File
import java.nio.ByteBuffer

/**
 * Checks the phone's video encoder without a camera, for the CI test on the emulator: encodes two
 * seconds of a made-up camera picture (grey with a red square at the top left, laid out as most
 * cameras give it: padded rows, V and U interleaved) and saves the stream to files/[FILE]. The
 * test then decodes it as the PC would and checks the picture came through.
 */
object EncoderSelfTest {
    const val FILE = "selftest.h264"
    private const val TAG = "HmdCamera"
    private const val WIDTH = 640
    private const val HEIGHT = 480
    private const val STRIDE = 704
    private const val FRAMES = 60

    /** Runs on the calling thread for a few seconds and logs how it went. */
    fun run(context: Context) {
        val stream = ByteArrayOutputStream()
        var frames = 0
        var keyframes = 0
        val encoder = try {
            VideoEncoder(WIDTH, HEIGHT, 30, 2_000_000) { _, keyframe, data, offset, length ->
                synchronized(stream) {
                    stream.write(data, offset, length)
                    frames++
                    if (keyframe) keyframes++
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Encoder self-test failed: no encoder", e)
            return
        }
        try {
            val planes = picture()
            for (i in 0 until FRAMES) {
                if (i == FRAMES / 2) encoder.requestKeyframe()
                val tag = VideoEncoder.Tag(i * 33_333_333L, 0, 0)
                var tries = 0
                while (!encoder.encode(planes, tag) && tries++ < 100) Thread.sleep(5)
                Thread.sleep(33) // a camera's pace
            }
            Thread.sleep(1000) // the last frames out
        } catch (e: Exception) {
            Log.e(TAG, "Encoder self-test failed", e)
            return
        } finally {
            encoder.close()
        }
        val bytes = synchronized(stream) { stream.toByteArray() }
        File(context.filesDir, FILE).writeBytes(bytes)
        val delay = encoder.delay.average(SystemClock.elapsedRealtimeNanos())
        Log.i(TAG, "Encoder self-test: $frames frames, $keyframes keyframes, ${bytes.size} bytes from ${encoder.name}, " +
            "bitrate mode ${encoder.bitrateMode.ifEmpty { "left to the encoder" }}, " +
            "${if (encoder.askedToHoldNoFrames) "asked" else "not asked"} to hold no frames, " +
            "delay ${delay?.let { "%.0f ms".format(it) } ?: "unknown"}")
    }

    // Planes Y, U, V: grey, with a red square of 160x120 at the top left.
    private fun picture(): Array<YuvPlane> {
        val y = ByteBuffer.allocateDirect(STRIDE * (HEIGHT - 1) + WIDTH)
        for (r in 0 until HEIGHT) {
            for (c in 0 until WIDTH) y.put(r * STRIDE + c, (if (r < 120 && c < 160) 91 else 126).toByte())
        }
        val vu = ByteBuffer.allocateDirect(STRIDE * (HEIGHT / 2 - 1) + WIDTH)
        for (r in 0 until HEIGHT / 2) {
            for (c in 0 until WIDTH / 2) {
                val red = r < 60 && c < 80
                vu.put(r * STRIDE + 2 * c, (if (red) 211 else 128).toByte())
                vu.put(r * STRIDE + 2 * c + 1, (if (red) 100 else 128).toByte())
            }
        }
        val u = vu.duplicate().apply { position(1) }.slice()
        return arrayOf(YuvPlane(y, STRIDE, 1), YuvPlane(u, STRIDE, 2), YuvPlane(vu, STRIDE, 2))
    }
}
