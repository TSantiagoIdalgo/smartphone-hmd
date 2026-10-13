package com.smartphonehmd.app

import android.app.Activity
import android.content.Context
import android.graphics.Color
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.text.InputType
import android.view.Gravity
import android.view.KeyEvent
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.TextView

/**
 * Enter the PC's IP and press Start. The phone then sends its sensors and shows the PC's video full
 * screen. Either volume button recenters the view, which works with the phone already inside the
 * headset. Back stops streaming.
 */
class MainActivity : Activity(), SurfaceHolder.Callback {
    private lateinit var streamer: SensorStreamer
    private lateinit var ipField: EditText
    private lateinit var startButton: Button
    private lateinit var statusText: TextView
    private lateinit var form: View
    private lateinit var videoView: SurfaceView
    private lateinit var videoStatus: TextView
    private var receiver: VideoReceiver? = null

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
        startButton = Button(this).apply {
            text = "Iniciar"
            setOnClickListener { startStreaming() }
        }
        statusText = TextView(this).apply { textSize = 16f }
        form = LinearLayout(this).apply {
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
        }

        videoView = SurfaceView(this).apply {
            visibility = View.GONE
            holder.addCallback(this@MainActivity)
        }
        videoStatus = TextView(this).apply {
            setTextColor(Color.WHITE)
            textSize = 16f
            gravity = Gravity.CENTER
            visibility = View.GONE
        }

        setContentView(FrameLayout(this).apply {
            setBackgroundColor(Color.BLACK)
            addView(videoView, FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT))
            addView(videoStatus, FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT))
            addView(form, FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT))
        })
        updateStatus()
    }

    override fun onResume() {
        super.onResume()
        if (streaming) streamer.start(pcIp())
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

    @Deprecated("Back stops streaming instead of closing the app")
    override fun onBackPressed() {
        if (streaming) stopStreaming() else super.onBackPressed()
    }

    // The video starts when the SurfaceView's surface exists, which happens once it becomes visible.
    override fun surfaceCreated(holder: SurfaceHolder) {
        receiver = VideoReceiver(holder.surface, streamer::requestKeyframe).also { it.start() }
    }

    override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {}

    override fun surfaceDestroyed(holder: SurfaceHolder) {
        receiver?.stop()
        receiver = null
    }

    private fun startStreaming() {
        val ip = pcIp()
        if (ip.isEmpty()) {
            statusText.text = "Escribe la IP de la PC"
            return
        }
        prefs().edit().putString(KEY_PC_IP, ip).apply()
        streaming = true
        streamer.start(ip)
        form.visibility = View.GONE
        videoView.visibility = View.VISIBLE
        videoStatus.visibility = View.VISIBLE
        enterFullScreen()
        updateStatus()
    }

    private fun stopStreaming() {
        streaming = false
        streamer.stop()
        videoView.visibility = View.GONE
        videoStatus.visibility = View.GONE
        form.visibility = View.VISIBLE
        updateStatus()
    }

    @Suppress("DEPRECATION")
    private fun enterFullScreen() {
        window.decorView.systemUiVisibility = View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY or
            View.SYSTEM_UI_FLAG_FULLSCREEN or
            View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or
            View.SYSTEM_UI_FLAG_LAYOUT_STABLE or
            View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION or
            View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
    }

    private fun updateStatus() {
        val error = streamer.lastError ?: receiver?.lastError
        if (!streaming) {
            statusText.text = if (error != null) "Error: $error" else "Escribe la IP de la PC y pulsa Iniciar"
            return
        }
        val frames = receiver?.framesShown?.get() ?: 0
        videoStatus.visibility = if (frames == 0 || error != null) View.VISIBLE else View.GONE
        videoStatus.text = when {
            error != null -> "Error: $error"
            else -> "Enviando sensores a ${pcIp()} (${streamer.packetsSent.get()} paquetes)\n" +
                "Esperando el video de SteamVR…\nAtrás: detener · Volumen: recentrar"
        }
    }

    private fun pcIp() = ipField.text.toString().trim()

    private fun prefs() = getSharedPreferences("settings", MODE_PRIVATE)

    private companion object {
        const val KEY_PC_IP = "pc_ip"
    }
}
