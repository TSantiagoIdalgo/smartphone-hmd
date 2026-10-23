package com.smartphonehmd.camera

import org.junit.Assert.assertEquals
import org.junit.Test

class RateMeterTest {
    private val start = 10_000_000_000L

    @Test
    fun countsEventsPerSecond() {
        val meter = RateMeter()
        for (i in 0..90) meter.tick(start + i * 33_333_333L)
        assertEquals(30f, meter.perSecond(start + 3_000_000_000L), 0.1f)
    }

    @Test
    fun dropsToZeroWhenEventsStop() {
        val meter = RateMeter()
        for (i in 0..60) meter.tick(start + i * 16_666_667L)
        assertEquals(60f, meter.perSecond(start + 1_000_000_000L), 0.1f)
        assertEquals(0f, meter.perSecond(start + 4_000_000_000L), 0f)
    }

    @Test
    fun aPauseDoesNotDragTheNextRateDown() {
        val meter = RateMeter()
        for (i in 0..45) meter.tick(start + i * 33_333_333L)
        val later = start + 10_000_000_000L
        for (i in 0..20) meter.tick(later + i * 33_333_333L)
        assertEquals(30f, meter.perSecond(later + 20 * 33_333_333L), 0.1f)
    }
}
