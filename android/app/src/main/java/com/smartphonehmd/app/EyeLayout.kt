package com.smartphonehmd.app

/**
 * Where each eye's image goes on the phone screen. Sizes are in millimetres so they can be matched
 * to the headset's lenses with a ruler, whatever the phone. Each eye is a rectangle centred on its
 * lens; [k1] and [k2] bend the image so that the lens' own distortion straightens it again.
 */
data class EyeLayout(
    val widthMm: Float,
    val heightMm: Float,
    /** Distance between the centres of both eyes. */
    val separationMm: Float,
    /** How far both centres sit above (+) or below (-) the middle of the screen. */
    val offsetMm: Float,
    val k1: Float,
    val k2: Float,
) {
    /** Rectangle of [eye] (0 = left, 1 = right) in pixels, with y growing upward as in OpenGL. */
    fun eyeRect(eye: Int, screenWidthPx: Int, screenHeightPx: Int, pxPerMm: Float): EyeRect {
        val side = if (eye == 0) -1f else 1f
        val centerX = screenWidthPx / 2f + side * separationMm / 2f * pxPerMm
        val centerY = screenHeightPx / 2f + offsetMm * pxPerMm
        val halfWidth = widthMm / 2f * pxPerMm
        val halfHeight = heightMm / 2f * pxPerMm
        return EyeRect(centerX - halfWidth, centerY - halfHeight, centerX + halfWidth, centerY + halfHeight)
    }

    companion object {
        /** One eye on each half of the screen, filling it: how the video looks before any adjustment. */
        fun fullScreen(screenWidthMm: Float, screenHeightMm: Float) = EyeLayout(
            widthMm = screenWidthMm / 2f,
            heightMm = screenHeightMm,
            separationMm = screenWidthMm / 2f,
            offsetMm = 0f,
            k1 = 0f,
            k2 = 0f,
        )
    }
}

data class EyeRect(val left: Float, val bottom: Float, val right: Float, val top: Float)
