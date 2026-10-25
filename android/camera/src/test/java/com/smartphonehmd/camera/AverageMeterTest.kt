package com.smartphonehmd.camera

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class AverageMeterTest {
    private val start = 10_000_000_000L

    @Test
    fun averagesOverASecond() {
        val meter = AverageMeter()
        for (i in 0..31) meter.add(start + i * 33_333_333L, if (i % 2 == 0) 40f else 60f)
        assertEquals(50f, meter.average(start + 31 * 33_333_333L)!!, 0.01f)
    }

    @Test
    fun nothingBeforeTheFirstSecond() {
        val meter = AverageMeter()
        for (i in 0..10) meter.add(start + i * 33_333_333L, 40f)
        assertNull(meter.average(start + 10 * 33_333_333L))
    }

    @Test
    fun nothingOnceTheValuesStop() {
        val meter = AverageMeter()
        for (i in 0..60) meter.add(start + i * 33_333_333L, 40f)
        assertEquals(40f, meter.average(start + 2_000_000_000L)!!, 0f)
        assertNull(meter.average(start + 5_000_000_000L))
    }

    @Test
    fun aPauseStartsAfresh() {
        val meter = AverageMeter()
        for (i in 0..60) meter.add(start + i * 33_333_333L, 100f)
        val later = start + 10_000_000_000L
        for (i in 0..10) meter.add(later + i * 33_333_333L, 20f)
        assertNull("the old average is gone", meter.average(later + 10 * 33_333_333L))
        for (i in 11..40) meter.add(later + i * 33_333_333L, 20f)
        assertEquals(20f, meter.average(later + 40 * 33_333_333L)!!, 0f)
    }
}
