package com.smartphonehmd.camera

/**
 * The average of a value, such as a delay, over the last second or so. Thread safe; times are in
 * nanoseconds.
 */
class AverageMeter {
    private var sum = 0.0
    private var count = 0
    private var windowStartNs = 0L
    private var lastAddNs = 0L
    private var average: Float? = null

    @Synchronized
    fun add(nowNs: Long, value: Float) {
        if (nowNs - lastAddNs > STALE_NS) {
            // The first value, or the first after a pause: older ones say nothing about now.
            sum = 0.0
            count = 0
            windowStartNs = nowNs
            average = null
        }
        lastAddNs = nowNs
        sum += value
        count++
        if (nowNs - windowStartNs >= WINDOW_NS) {
            average = (sum / count).toFloat()
            sum = 0.0
            count = 0
            windowStartNs = nowNs
        }
    }

    /** The average, or null before a second of values or once they have stopped. */
    @Synchronized
    fun average(nowNs: Long): Float? = if (nowNs - lastAddNs > STALE_NS) null else average

    private companion object {
        const val WINDOW_NS = 1_000_000_000L
        const val STALE_NS = 2_000_000_000L
    }
}
