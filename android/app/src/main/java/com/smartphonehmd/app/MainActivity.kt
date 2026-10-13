package com.smartphonehmd.app

import android.app.Activity
import android.content.Context
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.text.InputType
import android.view.Gravity
import android.view.KeyEvent
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.TextView

/**
 * Enter the PC's IP and press Start. While streaming, either volume button recenters the view,
 * which works with the phone already inside the headset.
 */
class MainActivity : Activity() {
    private lateinit var streamer: SensorStreamer
    private lateinit var ipField: EditText
    private lateinit var startButton: Button
    private lateinit var statusText: TextView

    private var streaming = false
    private val uiHandler = Handler(Looper.getMainLooper())
    private val statusUpdater = object : Runnable {
        override fun run() {
            updateStatus()
            uiHandler.postDelayed(this, 500)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        streamer = SensorStreamer(getSystemService(Context.SENSOR_SERVICE) as SensorManager)

        ipField = EditText(this).apply {
            hint = "IP de la PC, por ejemplo 192.168.1.10"
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            setSingleLine()
            setText(prefs().getString(KEY_PC_IP, ""))
        }
        startButton = Button(this).apply { setOnClickListener { toggleStreaming() } }
        statusText = TextView(this).apply { textSize = 16f }

        setContentView(LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER
            setPadding(48, 48, 48, 48)
            addView(TextView(context).apply {
                text = "Smartphone HMD"
                textSize = 24f
            })
            addView(ipField)
            addView(startButton)
            addView(statusText)
        })
        updateStatus()
    }

    override fun onResume() {
        super.onResume()
        if (streaming) streamer.start(ipField.text.toString().trim())
        uiHandler.post(statusUpdater)
    }

    override fun onPause() {
        super.onPause()
        streamer.stop()
        uiHandler.removeCallbacks(statusUpdater)
    }

    override fun onKeyDown(keyCode: Int, event: KeyEvent): Boolean {
        if (streaming && (keyCode == KeyEvent.KEYCODE_VOLUME_UP || keyCode == KeyEvent.KEYCODE_VOLUME_DOWN)) {
            if (event.repeatCount == 0) streamer.recenter()
            return true
        }
        return super.onKeyDown(keyCode, event)
    }

    private fun toggleStreaming() {
        if (streaming) {
            streaming = false
            streamer.stop()
        } else {
            val ip = ipField.text.toString().trim()
            if (ip.isEmpty()) {
                statusText.text = "Escribe la IP de la PC"
                return
            }
            prefs().edit().putString(KEY_PC_IP, ip).apply()
            streaming = true
            streamer.start(ip)
        }
        updateStatus()
    }

    private fun updateStatus() {
        startButton.text = if (streaming) "Detener" else "Iniciar"
        ipField.isEnabled = !streaming
        val error = streamer.lastError
        statusText.text = when {
            error != null -> "Error: $error"
            streaming -> "Enviando a ${ipField.text.toString().trim()}:${Protocol.SENSOR_PORT} · " +
                "${streamer.packetsSent.get()} paquetes\nBotón de volumen: recentrar"
            else -> "Detenido"
        }
    }

    private fun prefs() = getSharedPreferences("settings", MODE_PRIVATE)

    private companion object {
        const val KEY_PC_IP = "pc_ip"
    }
}
