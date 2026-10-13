package com.smartphonehmd.app

import android.media.MediaCodec
import android.media.MediaFormat
import android.os.Build
import android.os.SystemClock
import android.view.Surface
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.SocketTimeoutException
import java.util.concurrent.atomic.AtomicInteger

/**
 * Receives the PC's H.264 video over UDP and shows it on [surface]. When a frame is lost it waits
 * for the next keyframe and asks the PC for one through [requestKeyframe].
 */
class VideoReceiver(
    private val surface: Surface,
    private val requestKeyframe: () -> Unit,
) {
    @Volatile private var running = false
    private var thread: Thread? = null
    @Volatile private var socket: DatagramSocket? = null
    private var codec: MediaCodec? = null
    private var codecWidth = 0
    private var codecHeight = 0
    private var waitingForKeyframe = true
    private var lastKeyframeRequestMs = 0L

    val framesShown = AtomicInteger(0)
    @Volatile var lastError: String? = null
        private set

    fun start() {
        running = true
        thread = Thread(::receiveLoop, "video-receiver").apply { start() }
    }

    fun stop() {
        running = false
        socket?.close()
        thread?.join(1000)
        thread = null
    }

    private fun receiveLoop() {
        val assembler = FrameAssembler(::onFrame, onLoss = { waitingForKeyframe = true })
        try {
            DatagramSocket(VIDEO_PORT).use { s ->
                socket = s
                s.receiveBufferSize = 4 * 1024 * 1024
                s.soTimeout = 100
                val packet = DatagramPacket(ByteArray(2048), 2048)
                while (running) {
                    try {
                        s.receive(packet)
                        assembler.addPacket(packet.data, packet.length)
                    } catch (_: SocketTimeoutException) {
                    }
                    if (waitingForKeyframe) askForKeyframe()
                    drainOutput()
                }
            }
        } catch (e: Exception) {
            if (running) lastError = e.message ?: e.toString()
        } finally {
            releaseCodec()
        }
    }

    private fun askForKeyframe() {
        val now = SystemClock.elapsedRealtime()
        if (now - lastKeyframeRequestMs > KEYFRAME_REQUEST_INTERVAL_MS) {
            lastKeyframeRequestMs = now
            requestKeyframe()
        }
    }

    private fun onFrame(data: ByteArray, size: Int, keyframe: Boolean, width: Int, height: Int) {
        if (waitingForKeyframe && !keyframe) return
        waitingForKeyframe = false

        if (codec == null || width != codecWidth || height != codecHeight) {
            if (!keyframe) return
            if (!configureCodec(width, height)) return
        }
        val decoder = codec ?: return
        try {
            val index = decoder.dequeueInputBuffer(INPUT_TIMEOUT_US)
            if (index < 0) {
                waitingForKeyframe = true // decoder is backed up; skipping a frame breaks the chain
                return
            }
            decoder.getInputBuffer(index)?.apply {
                clear()
                put(data, 0, size)
            }
            decoder.queueInputBuffer(index, 0, size, SystemClock.elapsedRealtimeNanos() / 1000, 0)
        } catch (e: Exception) {
            lastError = e.message ?: e.toString()
            releaseCodec()
            waitingForKeyframe = true
        }
        drainOutput()
    }

    private fun configureCodec(width: Int, height: Int): Boolean {
        releaseCodec()
        return try {
            val format = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, width, height)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) format.setInteger(MediaFormat.KEY_LOW_LATENCY, 1)
            val decoder = MediaCodec.createDecoderByType(MediaFormat.MIMETYPE_VIDEO_AVC)
            decoder.configure(format, surface, null, 0)
            decoder.start()
            codec = decoder
            codecWidth = width
            codecHeight = height
            true
        } catch (e: Exception) {
            lastError = e.message ?: e.toString()
            false
        }
    }

    private fun drainOutput() {
        val decoder = codec ?: return
        val info = MediaCodec.BufferInfo()
        try {
            while (true) {
                val index = decoder.dequeueOutputBuffer(info, 0)
                if (index >= 0) {
                    decoder.releaseOutputBuffer(index, true)
                    framesShown.incrementAndGet()
                } else if (index == MediaCodec.INFO_TRY_AGAIN_LATER) {
                    break
                }
            }
        } catch (e: Exception) {
            lastError = e.message ?: e.toString()
        }
    }

    private fun releaseCodec() {
        codec?.let {
            try {
                it.stop()
            } catch (_: Exception) {
            }
            it.release()
        }
        codec = null
    }

    companion object {
        const val VIDEO_PORT = 9945
        private const val INPUT_TIMEOUT_US = 5_000L
        private const val KEYFRAME_REQUEST_INTERVAL_MS = 300L
    }
}
