package com.smartphonehmd.camera

import java.nio.ByteBuffer
import org.junit.Assert.assertEquals
import org.junit.Test

class YuvTest {
    private val width = 6
    private val height = 4

    private fun y(r: Int, c: Int) = (r * 16 + c).toByte()
    private fun u(r: Int, c: Int) = (100 + r * 8 + c).toByte()
    private fun v(r: Int, c: Int) = (200 + r * 8 + c).toByte()

    // As most cameras give it: rows padded to 8 bytes, V and U interleaved (NV21).
    private fun cameraPicture(): Array<YuvPlane> {
        val stride = 8
        val luma = ByteBuffer.allocateDirect(stride * height)
        val vu = ByteBuffer.allocateDirect(stride * height / 2)
        for (r in 0 until height) for (c in 0 until width) luma.put(r * stride + c, y(r, c))
        for (r in 0 until height / 2) {
            for (c in 0 until width / 2) {
                vu.put(r * stride + 2 * c, v(r, c))
                vu.put(r * stride + 2 * c + 1, u(r, c))
            }
        }
        val uPlane = vu.duplicate().apply { position(1) }.slice()
        return arrayOf(YuvPlane(luma, stride, 1), YuvPlane(uPlane, stride, 2), YuvPlane(vu, stride, 2))
    }

    private fun assertPicture(planes: Array<YuvPlane>) {
        fun at(plane: YuvPlane, r: Int, c: Int) = plane.buffer.get(r * plane.rowStride + c * plane.pixelStride)
        for (r in 0 until height) for (c in 0 until width) assertEquals("Y $r,$c", y(r, c), at(planes[0], r, c))
        for (r in 0 until height / 2) {
            for (c in 0 until width / 2) {
                assertEquals("U $r,$c", u(r, c), at(planes[1], r, c))
                assertEquals("V $r,$c", v(r, c), at(planes[2], r, c))
            }
        }
    }

    @Test
    fun cameraToPlanarEncoder() {
        val target = Yuv.layout(ByteBuffer.allocate(width * height * 3 / 2), width, height, semiPlanar = false)
        Yuv.copy(cameraPicture(), target, width, height)
        assertPicture(target)
    }

    @Test
    fun cameraToSemiPlanarEncoderWithUFirst() {
        // NV12 holds U and V the other way round from the camera: neither may overwrite the other.
        val buffer = ByteBuffer.allocate(width * height * 3 / 2)
        val target = Yuv.layout(buffer, width, height, semiPlanar = true)
        Yuv.copy(cameraPicture(), target, width, height)
        assertPicture(target)
        assertEquals(u(0, 0), buffer.get(width * height))
        assertEquals(v(0, 0), buffer.get(width * height + 1))
    }

    @Test
    fun planarLayoutPacksThePlanesTightly() {
        val buffer = ByteBuffer.allocate(width * height * 3 / 2)
        val planes = Yuv.layout(buffer, width, height, semiPlanar = false)
        Yuv.copy(cameraPicture(), planes, width, height)
        assertEquals(y(3, 5), buffer.get(width * height - 1))
        assertEquals(u(0, 0), buffer.get(width * height))
        assertEquals(v(0, 0), buffer.get(width * height + 6))
        assertEquals(v(1, 2), buffer.get(width * height * 3 / 2 - 1))
    }

    @Test
    fun copiesBetweenPaddedPlanarPictures() {
        val source = Yuv.layout(ByteBuffer.allocate(width * height * 3 / 2), width, height, semiPlanar = false)
        Yuv.copy(cameraPicture(), source, width, height)
        val padded = arrayOf(
            YuvPlane(ByteBuffer.allocate(16 * height), 16, 1),
            YuvPlane(ByteBuffer.allocate(8 * height / 2), 8, 1),
            YuvPlane(ByteBuffer.allocate(8 * height / 2), 8, 1),
        )
        Yuv.copy(source, padded, width, height)
        assertPicture(padded)
    }
}
