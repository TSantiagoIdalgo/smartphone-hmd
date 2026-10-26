package com.smartphonehmd.camera

/**
 * The fixed exposure a locked camera switches to: no longer than [MAX_EXPOSURE_NS], with the gain
 * raised to keep the picture as bright as the automatic exposure had it.
 *
 * A frame's time is when its exposure starts, and it reaches the app only once that exposure is
 * over: indoors at 30 fps the automatic exposure keeps it open up to 33 ms, all of it delay, and
 * a foot or the Move's bulb smears across the picture meanwhile. A shorter one costs some noise.
 */
object ShortExposure {
    const val MAX_EXPOSURE_NS = 10_000_000L

    class Setting(val exposureNs: Long, val iso: Int)

    /**
     * From the automatic [exposureNs] and [iso]: the shorter exposure, and the gain that makes up
     * for it within [maxIso]; as short as that gain allows when it cannot make up for all of it.
     */
    fun choose(exposureNs: Long, iso: Int, maxIso: Int, minExposureNs: Long): Setting {
        if (exposureNs <= MAX_EXPOSURE_NS || iso <= 0) return Setting(exposureNs, iso)
        val light = exposureNs.toDouble() * iso
        val shortest = maxOf(MAX_EXPOSURE_NS, minExposureNs, (light / maxOf(maxIso, iso)).toLong())
        val exposure = minOf(exposureNs, shortest)
        return Setting(exposure, minOf(maxIso.coerceAtLeast(iso), Math.round(light / exposure).toInt()))
    }
}
