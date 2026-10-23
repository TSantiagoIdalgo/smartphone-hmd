package com.smartphonehmd.camera

/** Just enough of H.264's byte stream: NAL units, each after a 00 00 01 start code. */
object H264 {
    private const val NAL_SLICE = 1
    private const val NAL_IDR_SLICE = 5
    private const val NAL_SPS = 7

    /** True if the frame in [data] carries the stream's headers (its SPS) before its picture. */
    fun hasHeaders(data: ByteArray, offset: Int, length: Int): Boolean {
        val end = offset + length
        var i = offset
        while (i + 3 < end) {
            if (data[i].toInt() == 0 && data[i + 1].toInt() == 0 && data[i + 2].toInt() == 1) {
                when (data[i + 3].toInt() and 0x1F) {
                    NAL_SPS -> return true
                    NAL_SLICE, NAL_IDR_SLICE -> return false
                }
                i += 3
            } else {
                i++
            }
        }
        return false
    }
}
