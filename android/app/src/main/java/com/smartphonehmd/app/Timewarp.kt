package com.smartphonehmd.app

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Turns each video frame by however much the head turned since the PC drew it, just before it is
 * shown (timewarp). The frame reaches the screen some 100 ms after the PC took the head's rotation
 * for it; turned here with the phone's own gyroscope, the view follows the head with the screen's
 * own delay instead, and keeps following it while the next frame is late.
 *
 * The PC says with each frame which sensor packet it had and how the head was turned for the frame
 * from that packet's rotation ([onFrame]). This keeps the rotation of every packet it sends
 * ([onGyro]), integrated from the same gyroscope, so the turn from the frame to now needs nothing
 * else. Head axes as the PC's: +x right, +y up, +z back; the phone sits in landscape with its top on
 * the left, so its gyroscope's (x, y, z) are the head's (y, -x, z). Thread-safe.
 */
class Timewarp {
    @Volatile var enabled = true

    // Head rotation (w, x, y, z) after each sensor packet, by the packet's seq.
    private val seqs = IntArray(SIZE)
    private val rotations = FloatArray(SIZE * 4)
    private var w = 1.0
    private var x = 0.0
    private var y = 0.0
    private var z = 0.0
    private val rate = DoubleArray(3)
    private var lastSampleNs = 0L

    // Each frame in the decoder: its time stamp there, the PC's packet seq and turn, how wide it is
    // drawn and how much of it is shown.
    private val frameTimesUs = LongArray(FRAMES) { -1L }
    private val frameSeqs = IntArray(FRAMES) { -1 }
    private val frameTurns = FloatArray(FRAMES * 4)
    private val frameTans = FloatArray(FRAMES * 2)
    private var nextFrame = 0

    /** Each gyroscope sample, as sent to the PC in packet [seq]. */
    @Synchronized
    fun onGyro(seq: Int, timestampNs: Long, gyro: FloatArray) {
        val rx = -gyro[1].toDouble()
        val ry = gyro[0].toDouble()
        val rz = gyro[2].toDouble()
        val dt = (timestampNs - lastSampleNs) / 1e9
        if (lastSampleNs != 0L && dt > 0.0 && dt < MAX_STEP_SECONDS) {
            turn(rx, ry, rz, dt)
        }
        lastSampleNs = timestampNs
        rate[0] = rx
        rate[1] = ry
        rate[2] = rz
        val i = seq and (SIZE - 1)
        seqs[i] = seq and SEQ_MASK
        rotations[i * 4] = w.toFloat()
        rotations[i * 4 + 1] = x.toFloat()
        rotations[i * 4 + 2] = y.toFloat()
        rotations[i * 4 + 3] = z.toFloat()
    }

    /**
     * A frame given to the decoder with time stamp [timeUs]; [turn] null if the PC did not say.
     * [tanDrawn] and [tanShown]: the tangents of half the angle each eye's frame spans and shows.
     */
    @Synchronized
    fun onFrame(timeUs: Long, seq: Int, turn: FloatArray?, tanDrawn: Float, tanShown: Float) {
        val i = nextFrame
        nextFrame = (nextFrame + 1) % FRAMES
        frameTimesUs[i] = timeUs
        frameSeqs[i] = if (turn != null) seq else -1
        if (turn != null) turn.copyInto(frameTurns, i * 4)
        frameTans[i * 2] = tanDrawn
        frameTans[i * 2 + 1] = tanShown
    }

    /**
     * The turn to show the frame with time stamp [frameTimeNs] at [nowNs] (elapsedRealtimeNanos):
     * fills [matrix] (3 x 3, column by column) with what takes a direction from the head as it will
     * be on screen to the head the frame was drawn for, and [tans] with the tangents of half the
     * angle the frame spans and shows. When the frame is not known, the identity and 1, 1.
     */
    @Synchronized
    fun matrix(frameTimeNs: Long, nowNs: Long, matrix: FloatArray, tans: FloatArray) {
        identity(matrix)
        tans[0] = 1f
        tans[1] = 1f
        // The decoder should keep each frame's time stamp; the nearest within a millisecond, should
        // it round it.
        val frameUs = frameTimeNs / 1000
        var f = -1
        for (i in 0 until FRAMES) {
            val off = abs(frameTimesUs[i] - frameUs)
            if (frameTimesUs[i] >= 0 && off <= MATCH_US && (f < 0 || off < abs(frameTimesUs[f] - frameUs))) f = i
        }
        if (f < 0) return
        if (frameTans[f * 2] > 0f && frameTans[f * 2 + 1] > 0f) {
            tans[0] = frameTans[f * 2]
            tans[1] = frameTans[f * 2 + 1]
        }
        val seq = frameSeqs[f]
        val p = seq and (SIZE - 1)
        if (!enabled || seq < 0 || seqs[p] != seq) return
        // Where the head was for the frame: its rotation at that packet, turned as the PC says.
        val a = Quat(rotations[p * 4], rotations[p * 4 + 1], rotations[p * 4 + 2], rotations[p * 4 + 3])
        val drawn = a * Quat(frameTurns[f * 4], frameTurns[f * 4 + 1], frameTurns[f * 4 + 2], frameTurns[f * 4 + 3])
        // Where it will be on screen: the newest, carried on to when the screen shows it.
        val ahead = ((nowNs - lastSampleNs) / 1e9).coerceIn(0.0, MAX_AHEAD_SECONDS) + SCREEN_SECONDS
        val shown = Quat(w, x, y, z) * Quat.turn(rate[0] * ahead, rate[1] * ahead, rate[2] * ahead)
        (drawn.conjugate() * shown).toMatrix(matrix)
    }

    private fun turn(rx: Double, ry: Double, rz: Double, dt: Double) {
        val q = Quat(w, x, y, z) * Quat.turn(rx * dt, ry * dt, rz * dt)
        val n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
        w = q.w / n
        x = q.x / n
        y = q.y / n
        z = q.z / n
    }

    private fun identity(matrix: FloatArray) {
        matrix.fill(0f)
        matrix[0] = 1f
        matrix[4] = 1f
        matrix[8] = 1f
    }

    /** A rotation; `a * b` turns by b in a's own axes, then by a. */
    class Quat(val w: Double, val x: Double, val y: Double, val z: Double) {
        constructor(w: Float, x: Float, y: Float, z: Float) : this(w.toDouble(), x.toDouble(), y.toDouble(), z.toDouble())

        operator fun times(b: Quat) = Quat(
            w * b.w - x * b.x - y * b.y - z * b.z,
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w,
        )

        fun conjugate() = Quat(w, -x, -y, -z)

        /** Column by column, as OpenGL takes a mat3. */
        fun toMatrix(m: FloatArray) {
            m[0] = (1 - 2 * (y * y + z * z)).toFloat()
            m[1] = (2 * (x * y + w * z)).toFloat()
            m[2] = (2 * (x * z - w * y)).toFloat()
            m[3] = (2 * (x * y - w * z)).toFloat()
            m[4] = (1 - 2 * (x * x + z * z)).toFloat()
            m[5] = (2 * (y * z + w * x)).toFloat()
            m[6] = (2 * (x * z + w * y)).toFloat()
            m[7] = (2 * (y * z - w * x)).toFloat()
            m[8] = (1 - 2 * (x * x + y * y)).toFloat()
        }

        companion object {
            /** The turn by the rotation vector (ax, ay, az), radians. */
            fun turn(ax: Double, ay: Double, az: Double): Quat {
                val angle = sqrt(ax * ax + ay * ay + az * az)
                if (angle < 1e-12) return Quat(1.0, 0.0, 0.0, 0.0)
                val s = sin(angle / 2) / angle
                return Quat(cos(angle / 2), ax * s, ay * s, az * s)
            }
        }
    }

    private companion object {
        const val SIZE = 4096 // a power of two; about 8 s of packets at 500 Hz
        const val SEQ_MASK = 0xFFFFFF
        const val FRAMES = 16
        const val MATCH_US = 1000L
        const val MAX_STEP_SECONDS = 0.5
        const val MAX_AHEAD_SECONDS = 0.05
        // From drawing to the screen showing it: about one refresh.
        const val SCREEN_SECONDS = 0.016
    }
}
