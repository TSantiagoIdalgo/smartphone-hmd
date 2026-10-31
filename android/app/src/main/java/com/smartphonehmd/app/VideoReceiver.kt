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
 * for the next keyframe and asks the PC for one through [requestKeyframe]. Each frame shown is
 * timed with [meter], and what the PC says it was drawn for goes to [timewarp].
 */
class VideoReceiver(
    private val surface: Surface,
    private val meter: LatencyMeter,
    private val timewarp: Timewarp,
    private val requestKeyframe: () -> Unit,
) {
    @Volatile private var running = false
    private var thread: Thread? = null
    @Volatile private var socket: DatagramSocket? = null
    private var codec: MediaCodec? = null
    private var outputThread: Thread? = null
    @Volatile private var outputRunning = false
    @Volatile private var decoderFailed = false
    private var codecWidth = 0
    private var codecHeight = 0
    private var waitingForKeyframe = true
    private var lastKeyframeRequestMs = 0L
    // Sensor seq of the frames inside the decoder, by presentation time.
    private val queuedTimesUs = LongArray(QUEUED_FRAMES)
    private val queuedSeqs = IntArray(QUEUED_FRAMES)
    private var queuedNext = 0

    // Counters shown on screen while no video is playing, to tell where the video stops.
    val packetsReceived = AtomicInteger(0)
    val framesComplete = AtomicInteger(0)
    val framesLost = AtomicInteger(0)
    val keyframes = AtomicInteger(0)
    val framesShown = AtomicInteger(0)
    @Volatile var lastFrameShownMs = 0L
        private set
    @Volatile var decoderName: String? = null
        private set
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
        val assembler = FrameAssembler(::onFrame, onLoss = {
            framesLost.incrementAndGet()
            waitingForKeyframe = true
        })
        try {
            DatagramSocket(VIDEO_PORT).use { s ->
                socket = s
                s.receiveBufferSize = 4 * 1024 * 1024
                s.soTimeout = 100
                val packet = DatagramPacket(ByteArray(2048), 2048)
                while (running) {
                    try {
                        s.receive(packet)
                        packetsReceived.incrementAndGet()
                        assembler.addPacket(packet.data, packet.length)
                    } catch (_: SocketTimeoutException) {
                    }
                    if (decoderFailed) {
                        releaseCodec()
                        waitingForKeyframe = true
                    }
                    if (waitingForKeyframe) askForKeyframe()
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

    private fun onFrame(
        data: ByteArray, size: Int, keyframe: Boolean, width: Int, height: Int, sensorSeq: Int, turn: FloatArray?,
        tanDrawn: Float, tanShown: Float,
    ) {
        framesComplete.incrementAndGet()
        if (keyframe) keyframes.incrementAndGet()
        if (waitingForKeyframe && !keyframe) return
        waitingForKeyframe = false

        if (codec == null || width != codecWidth || height != codecHeight) {
            if (!keyframe || !configureCodec(width, height)) {
                waitingForKeyframe = true
                return
            }
        }
        val decoder = codec ?: return
        try {
            // Give a busy decoder some time: skipping a frame breaks the chain until the next keyframe.
            val index = decoder.dequeueInputBuffer(INPUT_TIMEOUT_US)
            if (index < 0) {
                waitingForKeyframe = true
                return
            }
            decoder.getInputBuffer(index)?.apply {
                clear()
                put(data, 0, size)
            }
            // Render timestamps must use System.nanoTime's clock.
            val timeUs = System.nanoTime() / 1000
            synchronized(queuedTimesUs) {
                queuedTimesUs[queuedNext] = timeUs
                queuedSeqs[queuedNext] = sensorSeq
                queuedNext = (queuedNext + 1) % QUEUED_FRAMES
            }
            timewarp.onFrame(timeUs, sensorSeq, turn, tanDrawn, tanShown)
            decoder.queueInputBuffer(index, 0, size, timeUs, 0)
        } catch (e: Exception) {
            lastError = e.message ?: e.toString()
            releaseCodec()
            waitingForKeyframe = true
        }
    }

    private fun configureCodec(width: Int, height: Int): Boolean {
        releaseCodec()
        return try {
            val format = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, width, height)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) format.setInteger(MediaFormat.KEY_LOW_LATENCY, 1)
            format.setInteger(MediaFormat.KEY_PRIORITY, 0) // real time
            format.setInteger("vdec-lowlatency", 1) // MediaTek's own low-latency switch
            val decoder = MediaCodec.createDecoderByType(MediaFormat.MIMETYPE_VIDEO_AVC)
            decoder.configure(format, surface, null, 0)
            decoder.start()
            codec = decoder
            outputRunning = true
            outputThread = Thread({ outputLoop(decoder) }, "video-output").apply { start() }
            decoderName = decoder.name
            codecWidth = width
            codecHeight = height
            true
        } catch (e: Exception) {
            lastError = e.message ?: e.toString()
            false
        }
    }

    // Shows each frame as soon as the decoder finishes it, on its own thread. Waiting for it on the
    // receiving thread held every frame back until the next frame's first packet arrived.
    private fun outputLoop(decoder: MediaCodec) {
        val info = MediaCodec.BufferInfo()
        try {
            while (outputRunning) {
                val index = decoder.dequeueOutputBuffer(info, OUTPUT_TIMEOUT_US)
                if (index < 0) continue
                decoder.releaseOutputBuffer(index, true)
                val seq = synchronized(queuedTimesUs) {
                    val i = queuedTimesUs.indexOf(info.presentationTimeUs)
                    if (i >= 0) queuedSeqs[i] else null
                }
                if (seq != null) meter.shown(seq, SystemClock.elapsedRealtimeNanos())
                framesShown.incrementAndGet()
                lastFrameShownMs = SystemClock.elapsedRealtime()
                lastError = null
            }
        } catch (e: Exception) {
            // A decoder in an error state keeps throwing: the receiving thread starts again from the
            // next keyframe.
            if (outputRunning) {
                lastError = e.message ?: e.toString()
                decoderFailed = true
            }
        }
    }

    private fun releaseCodec() {
        outputRunning = false
        outputThread?.join(1000)
        outputThread = null
        decoderFailed = false
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
        private const val INPUT_TIMEOUT_US = 50_000L
        private const val OUTPUT_TIMEOUT_US = 10_000L
        private const val KEYFRAME_REQUEST_INTERVAL_MS = 300L
        private const val QUEUED_FRAMES = 16
    }
}
