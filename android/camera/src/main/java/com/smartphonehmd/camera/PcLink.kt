package com.smartphonehmd.camera

import android.os.SystemClock
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.NetworkInterface
import java.net.SocketTimeoutException
import java.util.Collections

/**
 * The phone's side of the link to the PC: finds the PC on every network the phone is on (Wi-Fi or
 * USB tethering) by broadcasting a discover packet, sends each frame's pose to it, and answers the
 * PC's clock pings at once. An IP typed in wins over the one found.
 */
class PcLink(private val deviceId: Int) {
    /** Typed in by the user; null or blank means search for the PC. */
    @Volatile var manualIp: String? = null

    /** Where the frames go now, as text for the screen, or null while searching. */
    val target: String? get() = targetAddress()?.hostAddress

    @Volatile var sentPackets = 0
        private set

    @Volatile private var foundAddress: InetAddress? = null
    @Volatile private var parsedIp: Pair<String, InetAddress?>? = null
    @Volatile private var socket: DatagramSocket? = null
    private val sendBuffer = ByteArray(BodyProtocol.BODY_PACKET_SIZE)

    fun start() {
        if (socket != null) return
        val s = try {
            DatagramSocket().apply { broadcast = true; soTimeout = 250 }
        } catch (_: Exception) {
            return
        }
        socket = s
        Thread({ receiveLoop(s) }, "pc-link").start()
    }

    /** Closes the socket, which also ends the receiving thread. */
    fun stop() {
        socket?.close()
        socket = null
    }

    /** Sends one frame's pose, if the PC is known. Called from the pose thread. */
    fun send(frame: BodyProtocol.Frame) {
        val s = socket ?: return
        val address = targetAddress() ?: return
        synchronized(sendBuffer) {
            val length = BodyProtocol.writeBody(sendBuffer, frame)
            try {
                s.send(DatagramPacket(sendBuffer, length, address, BodyProtocol.BODY_PORT))
                sentPackets++
            } catch (_: Exception) {
                // The network is changing; the next frame tries again.
            }
        }
    }

    // Parsed once per text typed, not once per frame: half-typed text can mean a name lookup.
    private fun targetAddress(): InetAddress? {
        val typed = manualIp?.trim()
        if (typed.isNullOrEmpty()) return foundAddress
        parsedIp?.let { if (it.first == typed) return it.second }
        val address = try {
            InetAddress.getByName(typed)
        } catch (_: Exception) {
            null
        }
        parsedIp = typed to address
        return address
    }

    private fun receiveLoop(s: DatagramSocket) {
        val discover = ByteArray(BodyProtocol.HEADER_SIZE)
        BodyProtocol.writeDiscover(discover)
        val pong = ByteArray(BodyProtocol.PONG_SIZE)
        val reply = DatagramPacket(ByteArray(256), 256)
        var nextDiscoverMs = 0L
        while (!s.isClosed) {
            val now = SystemClock.elapsedRealtime()
            if (now >= nextDiscoverMs) {
                // Keep looking even once found, slower, so a new address (cable unplugged) is followed.
                nextDiscoverMs = now + if (foundAddress == null) SEARCH_INTERVAL_MS else FOUND_INTERVAL_MS
                for (address in broadcastAddresses()) {
                    for (port in intArrayOf(BodyProtocol.BODY_PORT, BodyProtocol.DRIVER_PORT)) {
                        try {
                            s.send(DatagramPacket(discover, discover.size, address, port))
                        } catch (_: Exception) {
                        }
                    }
                }
            }
            try {
                reply.length = reply.data.size
                s.receive(reply)
            } catch (_: SocketTimeoutException) {
                continue
            } catch (_: Exception) {
                break // closed
            }
            when (BodyProtocol.typeOf(reply.data, reply.length)) {
                BodyProtocol.TYPE_PC_HERE -> foundAddress = reply.address
                BodyProtocol.TYPE_TIME_PING -> {
                    val phoneNs = SystemClock.elapsedRealtimeNanos()
                    BodyProtocol.writePong(
                        pong, BodyProtocol.seqOf(reply.data), phoneNs, BodyProtocol.timestampOf(reply.data), deviceId,
                    )
                    try {
                        s.send(DatagramPacket(pong, pong.size, reply.address, reply.port))
                    } catch (_: Exception) {
                    }
                }
            }
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
        const val SEARCH_INTERVAL_MS = 1000L
        const val FOUND_INTERVAL_MS = 5000L
    }
}
