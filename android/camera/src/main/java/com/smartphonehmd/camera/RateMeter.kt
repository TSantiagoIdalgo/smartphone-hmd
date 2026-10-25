package com.smartphonehmd.camera

/**
 * Events (or an amount, such as bytes) per second over the last second or so. Thread safe;
 * times are in nanoseconds.
 */
class RateMeter {
    private var total = 0L
    private var windowStartNs = 0L
    private var lastTickNs = 0L
    private var rate = 0f

    @Synchronized
    fun tick(nowNs: Long, amount: Long = 1) {
        val stale = nowNs - lastTickNs > STALE_NS
        lastTickNs = nowNs
        if (stale) {
            // First event, or the first after a pause: it starts the count, the ones after it count.
            total = 0
            windowStartNs = nowNs
            return
        }
        total += amount
        val elapsed = nowNs - windowStartNs
        if (elapsed >= WINDOW_NS) {
            rate = total * 1e9f / elapsed
            total = 0
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
