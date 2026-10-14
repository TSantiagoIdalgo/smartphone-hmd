package com.smartphonehmd.app

/**
 * Times the whole loop: a sensor packet leaves the phone, SteamVR renders a frame with it, and the
 * phone shows that frame. The PC puts the newest packet's seq (its low 24 bits) in each frame, so
 * both ends of the measurement use the phone's own clock. Thread-safe.
 */
class LatencyMeter {
    private val seqs = IntArray(SIZE)
    private val sentNs = LongArray(SIZE)
    private var averageNs = 0.0
    private var measured = false

    /** Call when the packet numbered [seq] is sent at [timeNs]. */
    @Synchronized
    fun sent(seq: Int, timeNs: Long) {
        val i = seq and (SIZE - 1)
        seqs[i] = seq and SEQ_MASK
        sentNs[i] = timeNs
    }

    /** Call when the frame that carried [frameSeq] is shown at [timeNs]. 0 means the PC did not say. */
    @Synchronized
    fun shown(frameSeq: Int, timeNs: Long) {
        if (frameSeq == 0) return
        val i = frameSeq and (SIZE - 1)
        if (seqs[i] != frameSeq) return // older than the ring remembers
        val latency = timeNs - sentNs[i]
        if (latency <= 0 || latency > MAX_NS) return
        averageNs = if (measured) averageNs + (latency - averageNs) * SMOOTHING else latency.toDouble()
        measured = true
    }

    /** Smoothed latency in milliseconds, or null until a frame has been timed. */
    @Synchronized
    fun averageMs(): Float? = if (measured) (averageNs / 1_000_000).toFloat() else null

    @Synchronized
    fun reset() {
        seqs.fill(0)
        measured = false
    }

    private companion object {
        const val SIZE = 4096 // a power of two; about 8 s of packets at 500 Hz
        const val SEQ_MASK = 0xFFFFFF
        const val MAX_NS = 1_000_000_000L
        const val SMOOTHING = 0.05
    }
}
