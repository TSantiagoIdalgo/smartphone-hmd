package com.smartphonehmd.camera

/** Events per second over the last second or so. Thread safe; times are in nanoseconds. */
class RateMeter {
    private var count = 0
    private var windowStartNs = 0L
    private var lastTickNs = 0L
    private var rate = 0f

    @Synchronized
    fun tick(nowNs: Long) {
        if (nowNs - lastTickNs > STALE_NS) {
            // First event, or the first after a pause: start counting afresh.
            count = 0
            windowStartNs = nowNs
        }
        lastTickNs = nowNs
        count++
        val elapsed = nowNs - windowStartNs
        if (elapsed >= WINDOW_NS) {
            rate = (count - 1) * 1e9f / elapsed
            count = 1
            windowStartNs = nowNs
        }
    }

    /** The rate, or 0 once the events have stopped. */
    @Synchronized
    fun perSecond(nowNs: Long): Float = if (nowNs - lastTickNs > STALE_NS) 0f else rate

    private companion object {
        const val WINDOW_NS = 1_000_000_000L
        const val STALE_NS = 2_000_000_000L
    }
}
