package com.smartphonehmd.camera

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class H264Test {
    private val sps = byteArrayOf(0, 0, 0, 1, 0x67, 0x42, 0, 0x1E)
    private val pps = byteArrayOf(0, 0, 0, 1, 0x68, 0xCE.toByte(), 0x38, 0x80.toByte())
    private val idr = byteArrayOf(0, 0, 1, 0x65, 0x88.toByte(), 0x84.toByte(), 0, 0x33)
    private val slice = byteArrayOf(0, 0, 0, 1, 0x41, 0x9A.toByte(), 0x21, 0x6C)

    @Test
    fun keyframeWithItsHeaders() {
        val frame = byteArrayOf(9, 9) + sps + pps + idr
        assertTrue(H264.hasHeaders(frame, 2, frame.size - 2))
    }

    @Test
    fun keyframeWithoutHeaders() {
        assertFalse(H264.hasHeaders(idr, 0, idr.size))
        assertFalse(H264.hasHeaders(slice, 0, slice.size))
    }

    @Test
    fun headersAfterThePictureDoNotCount() {
        val frame = idr + sps
        assertFalse(H264.hasHeaders(frame, 0, frame.size))
        assertFalse("cut before the NAL type", H264.hasHeaders(sps, 0, 4))
    }
}
