package com.smartphonehmd.camera

import java.nio.ByteBuffer

/** One plane of a YUV 4:2:0 picture: where its bytes are and how they are spaced. */
class YuvPlane(val buffer: ByteBuffer, val rowStride: Int, val pixelStride: Int)

/**
 * Copies YUV 4:2:0 pictures between the layouts cameras and encoders use: planes with their own
 * row padding, and U and V either apart (pixel stride 1) or interleaved (pixel stride 2).
 */
object Yuv {
    /** Copies the planes Y, U, V of a [width] x [height] picture from [source] to [target]. */
    fun copy(source: Array<YuvPlane>, target: Array<YuvPlane>, width: Int, height: Int) {
        val row = ByteArray(2 * width)
        val packed = ByteArray(width)
        for (p in 0..2) {
            val w = if (p == 0) width else (width + 1) / 2
            val h = if (p == 0) height else (height + 1) / 2
            copyPlane(source[p], target[p], w, h, row, packed)
        }
    }

    /**
     * The three planes of a picture packed tightly in [buffer]: planar (I420) or, if [semiPlanar],
     * with U and V interleaved (NV12). For encoders that hand out a plain buffer.
     */
    fun layout(buffer: ByteBuffer, width: Int, height: Int, semiPlanar: Boolean): Array<YuvPlane> {
        val ySize = width * height
        val chromaWidth = (width + 1) / 2
        val chromaSize = chromaWidth * ((height + 1) / 2)
        val y = YuvPlane(slice(buffer, 0), width, 1)
        return if (semiPlanar) {
            arrayOf(y, YuvPlane(slice(buffer, ySize), chromaWidth * 2, 2), YuvPlane(slice(buffer, ySize + 1), chromaWidth * 2, 2))
        } else {
            arrayOf(y, YuvPlane(slice(buffer, ySize), chromaWidth, 1), YuvPlane(slice(buffer, ySize + chromaSize), chromaWidth, 1))
        }
    }

    private fun slice(buffer: ByteBuffer, offset: Int): ByteBuffer =
        buffer.duplicate().apply { clear(); position(offset) }.slice()

    // A row at a time, gathered into [packed] when its pixels are spaced out. Interleaved U and V
    // are written pixel by pixel, so neither colour overwrites the other's bytes.
    private fun copyPlane(source: YuvPlane, target: YuvPlane, width: Int, height: Int, row: ByteArray, packed: ByteArray) {
        val from = source.buffer.duplicate()
        val to = target.buffer.duplicate()
        from.clear()
        to.clear()
        for (r in 0 until height) {
            from.position(r * source.rowStride)
            from.get(row, 0, (width - 1) * source.pixelStride + 1)
            val pixels = if (source.pixelStride == 1) {
                row
            } else {
                for (i in 0 until width) packed[i] = row[i * source.pixelStride]
                packed
            }
            if (target.pixelStride == 1) {
                to.position(r * target.rowStride)
                to.put(pixels, 0, width)
            } else {
                val at = r * target.rowStride
                for (i in 0 until width) to.put(at + i * target.pixelStride, pixels[i])
            }
        }
    }
}
