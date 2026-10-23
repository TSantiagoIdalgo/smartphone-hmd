package com.smartphonehmd.camera

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.view.View

/**
 * Draws the pose found on the camera preview: a dot per point, green when the model is sure the
 * point is in view, yellow when unsure, red when it is guessing. The preview fits the image inside
 * the view, so the image's rectangle is worked out the same way here.
 */
class PoseOverlay(context: Context) : View(context) {
    /** The front camera's preview is a mirror image; the points are not. */
    @Volatile var mirrored = false

    private val points = FloatArray(BodyProtocol.LANDMARK_COUNT * 5)
    private var count = 0
    private var imageWidth = 0
    private var imageHeight = 0
    private val dot = Paint(Paint.ANTI_ALIAS_FLAG)
    private val bone = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(200, 255, 255, 255)
        strokeWidth = 6f
    }

    /** Called from the pose thread with each frame's points; [landmarks] is copied. */
    fun show(landmarks: FloatArray, landmarkCount: Int, width: Int, height: Int) {
        synchronized(points) {
            landmarkCount.coerceIn(0, BodyProtocol.LANDMARK_COUNT).let { n ->
                System.arraycopy(landmarks, 0, points, 0, n * 5)
                count = n
            }
            imageWidth = width
            imageHeight = height
        }
        postInvalidateOnAnimation()
    }

    override fun onDraw(canvas: Canvas) {
        synchronized(points) {
            if (count == 0 || imageWidth == 0 || imageHeight == 0) return
            val scale = minOf(width.toFloat() / imageWidth, height.toFloat() / imageHeight)
            val drawnWidth = imageWidth * scale
            val drawnHeight = imageHeight * scale
            val left = (width - drawnWidth) / 2
            val top = (height - drawnHeight) / 2
            fun x(i: Int): Float = left + drawnWidth * (if (mirrored) 1 - points[i * 5] else points[i * 5])
            fun y(i: Int): Float = top + drawnHeight * points[i * 5 + 1]

            for (pair in BONES) {
                if (pair[0] < count && pair[1] < count) canvas.drawLine(x(pair[0]), y(pair[0]), x(pair[1]), y(pair[1]), bone)
            }
            for (i in 0 until count) {
                if (i in 1..10) continue // eyes, ears and mouth: the nose is enough
                val visibility = points[i * 5 + 3]
                dot.color = when {
                    visibility >= 0.8f -> Color.GREEN
                    visibility >= 0.5f -> Color.YELLOW
                    else -> Color.RED
                }
                canvas.drawCircle(x(i), y(i), if (i in 23..32) 14f else 9f, dot)
            }
        }
    }

    private companion object {
        // MediaPipe's pose points: 11/12 shoulders, 13/14 elbows, 15/16 wrists, 23/24 hips,
        // 25/26 knees, 27/28 ankles, 29/30 heels, 31/32 toes. Odd numbers are the person's left.
        val BONES = arrayOf(
            intArrayOf(11, 12), intArrayOf(11, 13), intArrayOf(13, 15), intArrayOf(12, 14), intArrayOf(14, 16),
            intArrayOf(11, 23), intArrayOf(12, 24), intArrayOf(23, 24),
            intArrayOf(23, 25), intArrayOf(25, 27), intArrayOf(27, 29), intArrayOf(29, 31), intArrayOf(27, 31),
            intArrayOf(24, 26), intArrayOf(26, 28), intArrayOf(28, 30), intArrayOf(30, 32), intArrayOf(28, 32),
        )
    }
}
