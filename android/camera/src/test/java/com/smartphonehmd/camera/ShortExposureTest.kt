package com.smartphonehmd.camera

import org.junit.Assert.assertEquals
import org.junit.Test

class ShortExposureTest {
    @Test
    fun shortensALongExposureAndMakesUpWithGain() {
        val s = ShortExposure.choose(exposureNs = 33_000_000L, iso = 200, maxIso = 3200, minExposureNs = 100_000L)
        assertEquals(10_000_000L, s.exposureNs)
        assertEquals(660, s.iso)
    }

    @Test
    fun leavesAShortExposureAlone() {
        val s = ShortExposure.choose(exposureNs = 8_000_000L, iso = 100, maxIso = 3200, minExposureNs = 100_000L)
        assertEquals(8_000_000L, s.exposureNs)
        assertEquals(100, s.iso)
    }

    @Test
    fun asShortAsTheGainAllowsInADarkRoom() {
        // 33 ms at ISO 1600 with ISO 3200 at most: 16.5 ms keeps the brightness.
        val s = ShortExposure.choose(exposureNs = 33_000_000L, iso = 1600, maxIso = 3200, minExposureNs = 100_000L)
        assertEquals(16_500_000L, s.exposureNs)
        assertEquals(3200, s.iso)
    }

    @Test
    fun neverLongerThanItWas() {
        // Already at the gain's top: nothing to trade.
        val s = ShortExposure.choose(exposureNs = 33_000_000L, iso = 3200, maxIso = 3200, minExposureNs = 100_000L)
        assertEquals(33_000_000L, s.exposureNs)
        assertEquals(3200, s.iso)
    }
}
