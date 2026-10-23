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
 * USB tethering) by broadcasting a discover packet, sends it the camera's video, and answers the
 * PC's clock pings at once. The poses the PC finds and its requests for a keyframe come back on
 * the link's thread. An IP typed in wins over the one found.
 */
class PcLink(private val deviceId: Int) {
    /** Called on the link's thread with each pose from the PC; the frame is reused afterwards. */
    @Volatile var onPose: ((BodyProtocol.Frame) -> Unit)? = null

    /** Called on the link's thread when the PC needs a frame it can decode on its own. */
    @Volatile var onKeyframeRequest: (() -> Unit)? = null

    /** Typed in by the user; null or blank means search for the PC. */
    @Volatile var manualIp: String? = null

    /** Where the frames go now, as text for the screen, or null while searching. */
    val target: String? get() = targetAddress()?.hostAddress

    @Volatile var sentPackets = 0
        private set

    private var sentFrames = 0

    @Volatile private var foundAddress: InetAddress? = null
    @Volatile private var parsedIp: Pair<String, InetAddress?>? = null
    @Volatile private var socket: DatagramSocket? = null
    private val sendBuffer = ByteArray(BodyProtocol.VIDEO_HEADER_SIZE + BodyProtocol.MAX_VIDEO_PAYLOAD)

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

    /**
     * Sends one encoded frame in pieces, if the PC is known; false if it is not. Called from the
     * encoder's thread.
     */
    fun sendVideo(
        tag: VideoEncoder.Tag, width: Int, height: Int, keyframe: Boolean, data: ByteArray, offset: Int, length: Int,
    ): Boolean {
        val s = socket ?: return false
        val address = targetAddress() ?: return false
        val flags = tag.flags or if (keyframe) BodyProtocol.FLAG_KEYFRAME else 0
        val pieces = maxOf(1, (length + BodyProtocol.MAX_VIDEO_PAYLOAD - 1) / BodyProtocol.MAX_VIDEO_PAYLOAD)
        synchronized(sendBuffer) {
            sentFrames++
            for (piece in 0 until pieces) {
                val at = piece * BodyProtocol.MAX_VIDEO_PAYLOAD
                val size = BodyProtocol.writeVideo(
                    sendBuffer, sentFrames, tag.captureNs, deviceId, width, height, piece, pieces, tag.quarterTurns, flags,
                    data, offset + at, minOf(BodyProtocol.MAX_VIDEO_PAYLOAD, length - at),
                )
                try {
                    s.send(DatagramPacket(sendBuffer, size, address, BodyProtocol.BODY_PORT))
                    sentPackets++
                } catch (_: Exception) {
                    // The network is changing; the PC asks for a keyframe once it is back.
                }
            }
        }
        return true
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
        val reply = DatagramPacket(ByteArray(1024), 1024)
        val pose = BodyProtocol.Frame()
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
                BodyProtocol.TYPE_BODY -> if (BodyProtocol.readBody(reply.data, reply.length, pose)) onPose?.invoke(pose)
                BodyProtocol.TYPE_KEYFRAME_REQUEST -> onKeyframeRequest?.invoke()
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
