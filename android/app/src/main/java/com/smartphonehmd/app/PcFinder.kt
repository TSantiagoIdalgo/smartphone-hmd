package com.smartphonehmd.app

import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.NetworkInterface
import java.net.SocketTimeoutException
import java.util.Collections
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Finds the PC without typing its IP: broadcasts a discover packet on every network the phone is
 * on (Wi-Fi and USB tethering alike) once a second, and the SteamVR driver answers. [onFound] gets
 * the PC's address on the main thread.
 */
class PcFinder(private val onFound: (String) -> Unit) {
    private val mainHandler = Handler(Looper.getMainLooper())
    private var running: AtomicBoolean? = null

    val isRunning get() = running?.get() == true

    fun start() {
        if (isRunning) return
        val flag = AtomicBoolean(true)
        running = flag
        Thread({ search(flag) }, "pc-finder").start()
    }

    /** Stops within a fraction of a second; no more results arrive after this call. */
    fun stop() {
        running?.set(false)
        running = null
    }

    private fun search(running: AtomicBoolean) {
        val request = ByteArray(Protocol.HEADER_SIZE)
        Protocol.writeCommand(request, Protocol.TYPE_DISCOVER, 0, 0)
        val reply = DatagramPacket(ByteArray(64), 64)
        try {
            DatagramSocket().use { socket ->
                socket.broadcast = true
                socket.soTimeout = 250
                var nextSendMs = 0L
                while (running.get()) {
                    val now = SystemClock.elapsedRealtime()
                    if (now >= nextSendMs) {
                        nextSendMs = now + INTERVAL_MS
                        for (address in broadcastAddresses()) {
                            try {
                                socket.send(DatagramPacket(request, request.size, address, Protocol.SENSOR_PORT))
                            } catch (_: Exception) {
                                // That network is going away; the others may still work.
                            }
                        }
                    }
                    try {
                        socket.receive(reply)
                    } catch (_: SocketTimeoutException) {
                        continue
                    }
                    val ip = reply.address?.hostAddress
                    if (ip != null && Protocol.isPcHere(reply.data, reply.length)) {
                        mainHandler.post { if (running.get()) onFound(ip) }
                    }
                }
            }
        } catch (_: Exception) {
            // No usable network right now. start() tries again later.
        } finally {
            running.set(false)
        }
    }

    // The general broadcast address, plus each network's own, because Android only sends the
    // general one on its default network and USB tethering never is.
    private fun broadcastAddresses(): List<InetAddress> {
        val addresses = mutableListOf(InetAddress.getByName("255.255.255.255"))
        try {
            for (network in Collections.list(NetworkInterface.getNetworkInterfaces() ?: return addresses)) {
                if (!network.isUp || network.isLoopback) continue
                network.interfaceAddresses.mapNotNullTo(addresses) { it.broadcast }
            }
        } catch (_: Exception) {
        }
        return addresses.distinct()
    }

    private companion object {
        const val INTERVAL_MS = 1000L
    }
}
