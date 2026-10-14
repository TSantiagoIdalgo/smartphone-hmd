package com.smartphonehmd.app

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class LatencyMeterTest {
    private val ms = 1_000_000L

    @Test
    fun measuresFromSendingTheSensorsToShowingTheirFrame() {
        val meter = LatencyMeter()
        meter.sent(10, 1000 * ms)
        meter.sent(11, 1002 * ms)
        meter.shown(11, 1042 * ms)
        assertEquals(40f, meter.averageMs()!!, 0.001f)
    }

    @Test
    fun smoothsLaterMeasurements() {
        val meter = LatencyMeter()
        meter.sent(1, 0)
        meter.shown(1, 40 * ms)
        meter.sent(2, 100 * ms)
        meter.shown(2, 160 * ms) // 60 ms moves the average 5% of the way
        assertEquals(41f, meter.averageMs()!!, 0.001f)
    }

    @Test
    fun ignoresFramesItCannotTime() {
        val meter = LatencyMeter()
        meter.sent(5, 1000 * ms)
        meter.shown(0, 1040 * ms) // the PC did not say
        meter.shown(6, 1040 * ms) // never sent
        meter.shown(5 + 4096, 1040 * ms) // same slot, but a seq the ring no longer holds
        meter.shown(5, 3000 * ms) // implausibly late
        assertNull(meter.averageMs())
    }

    @Test
    fun matchesTheLow24BitsThePcSendsBack() {
        val meter = LatencyMeter()
        val seq = 0x01ABCDEF
        meter.sent(seq, 0)
        meter.shown(seq and 0xFFFFFF, 30 * ms)
        assertEquals(30f, meter.averageMs()!!, 0.001f)
    }

    @Test
    fun resetForgetsEverything() {
        val meter = LatencyMeter()
        meter.sent(1, 0)
        meter.shown(1, 30 * ms)
        meter.reset()
        assertNull(meter.averageMs())
        meter.shown(1, 30 * ms)
        assertNull(meter.averageMs())
    }
}
