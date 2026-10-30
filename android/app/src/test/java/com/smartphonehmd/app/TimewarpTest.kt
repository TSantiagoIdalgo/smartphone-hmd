package com.smartphonehmd.app

import kotlin.math.cos
import kotlin.math.sin
import org.junit.Assert.assertEquals
import org.junit.Test

class TimewarpTest {
    private val timewarp = Timewarp()
    private val matrix = FloatArray(9)
    private var seq = 0
    private var timeNs = 1_000_000_000L

    /** The head turns left at [rate] rad/s (about +y) for [seconds], in samples 5 ms apart. */
    private fun turnLeft(rate: Float, seconds: Double): Int {
        repeat((seconds / 0.005).toInt()) {
            timeNs += 5_000_000L
            // The phone's x is the head's y.
            timewarp.onGyro(++seq, timeNs, floatArrayOf(rate, 0f, 0f))
        }
        return seq
    }

    /** Where the frame looks for what is straight ahead on screen now. */
    private fun ahead(): FloatArray = floatArrayOf(-matrix[6], -matrix[7], -matrix[8])

    @Test
    fun aFrameWithoutATurnIsShownAsItIs() {
        val packet = turnLeft(0f, 0.1)
        timewarp.onFrame(5L, packet, null, 1.25f)
        turnLeft(1f, 0.3)

        assertEquals(1.25f, timewarp.matrix(5_000L, timeNs, matrix), 0f)
        assertEquals(floatArrayOf(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f).toList(), matrix.toList())
    }

    @Test
    fun aFrameIsTurnedByWhatTheHeadTurnedSince() {
        // Drawn at a packet; then the head turns 0.3 rad to the left and stops. Straight ahead on
        // screen is now 0.3 rad left of what the frame has straight ahead.
        val packet = turnLeft(0f, 0.1)
        timewarp.onFrame(5L, packet, floatArrayOf(1f, 0f, 0f, 0f), 1f)
        turnLeft(0.3f, 1.0)
        turnLeft(0f, 0.01)

        assertEquals(1f, timewarp.matrix(5_000L, timeNs, matrix), 0f)
        val d = ahead()
        assertEquals(-sin(0.3).toFloat(), d[0], 1e-3f)
        assertEquals(0f, d[1], 1e-6f)
        assertEquals(-cos(0.3).toFloat(), d[2], 1e-3f)
    }

    @Test
    fun theTurnThePcDrewForCounts() {
        // The PC drew the frame 0.1 rad to the left of where the head was at the packet (its
        // prediction), and the head stayed: what is straight ahead is 0.1 rad right in the frame.
        val packet = turnLeft(0f, 0.1)
        timewarp.onFrame(5L, packet, floatArrayOf(cos(0.05).toFloat(), 0f, sin(0.05).toFloat(), 0f), 1f)
        turnLeft(0f, 0.1)

        timewarp.matrix(5_000L, timeNs, matrix)
        val d = ahead()
        assertEquals(sin(0.1).toFloat(), d[0], 1e-3f)
        assertEquals(-cos(0.1).toFloat(), d[2], 1e-3f)
    }

    @Test
    fun turnedOffItShowsTheFrameAsItIs() {
        val packet = turnLeft(0f, 0.1)
        timewarp.onFrame(5L, packet, floatArrayOf(1f, 0f, 0f, 0f), 1f)
        turnLeft(1f, 0.3)
        timewarp.enabled = false

        timewarp.matrix(5_000L, timeNs, matrix)
        assertEquals(floatArrayOf(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f).toList(), matrix.toList())
    }
}
