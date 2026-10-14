package com.smartphonehmd.app

import org.junit.Assert.assertEquals
import org.junit.Test

class EyeLayoutTest {
    // A 160 x 70 mm screen at 10 pixels per millimetre.
    private val width = 1600
    private val height = 700
    private val pxPerMm = 10f

    @Test
    fun fullScreenLayoutPutsOneEyeOnEachHalf() {
        val layout = EyeLayout.fullScreen(160f, 70f)

        assertEquals(EyeRect(0f, 0f, 800f, 700f), layout.eyeRect(0, width, height, pxPerMm))
        assertEquals(EyeRect(800f, 0f, 1600f, 700f), layout.eyeRect(1, width, height, pxPerMm))
    }

    @Test
    fun eyesAreCentredOnTheirLensesAndMoveUpWithTheOffset() {
        val layout = EyeLayout(widthMm = 50f, heightMm = 40f, separationMm = 60f, offsetMm = 5f, k1 = 0f, k2 = 0f)

        // Centres 30 mm either side of the middle and 5 mm above it.
        assertEquals(EyeRect(250f, 200f, 750f, 600f), layout.eyeRect(0, width, height, pxPerMm))
        assertEquals(EyeRect(850f, 200f, 1350f, 600f), layout.eyeRect(1, width, height, pxPerMm))
    }
}
