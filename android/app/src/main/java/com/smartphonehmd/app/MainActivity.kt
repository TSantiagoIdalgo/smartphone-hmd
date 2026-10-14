package com.smartphonehmd.app

import android.app.Activity
import android.content.Context
import android.graphics.Color
import android.hardware.SensorManager
import android.opengl.GLSurfaceView
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.text.InputType
import android.util.DisplayMetrics
import android.view.Gravity
import android.view.KeyEvent
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.TextView

/**
 * Enter the PC's IP and press Start. The phone then sends its sensors and shows the PC's video full
 * screen, each eye where the headset's lenses need it. Either volume button recenters the view,
 * which works with the phone already inside the headset. A tap opens the eye adjustment; Back
 * closes it or stops streaming.
 */
class MainActivity : Activity() {
    private lateinit var streamer: SensorStreamer
    private lateinit var ipField: EditText
    private lateinit var startButton: Button
    private lateinit var statusText: TextView
    private lateinit var form: View
    private lateinit var videoView: GLSurfaceView
    private lateinit var renderer: EyeRenderer
    private lateinit var adjustPanel: AdjustPanel
    private lateinit var videoStatus: TextView
    private var receiver: VideoReceiver? = null
    private var decoderTarget: DecoderTarget? = null

    private var streaming = false
    private var resumed = false
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
        // Use the whole screen, camera hole included, so the eyes centre on the physical screen.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            window.attributes.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES
        }
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

        val screen = screenMetrics()
        val pxPerMm = pixelsPerMm(screen)
        val defaults = EyeLayout.fullScreen(screen.widthPixels / pxPerMm, screen.heightPixels / pxPerMm)
        val layout = loadLayout(defaults)
        videoView = GLSurfaceView(this).apply {
            setEGLContextClientVersion(2)
            preserveEGLContextOnPause = true
            visibility = View.GONE
            setOnClickListener { if (streaming) setPanelOpen(adjustPanel.visibility != View.VISIBLE) }
        }
        renderer = EyeRenderer(videoView, pxPerMm, layout, ::onDecoderTarget)
        videoView.setRenderer(renderer)
        videoView.renderMode = GLSurfaceView.RENDERMODE_WHEN_DIRTY
        adjustPanel = AdjustPanel(
            this, defaults, layout,
            onChange = { renderer.layout = it },
            onGrid = { renderer.showGrid = it },
            onDone = { setPanelOpen(false) },
        )
        adjustPanel.visibility = View.GONE
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
            addView(adjustPanel, FrameLayout.LayoutParams(screen.widthPixels / 2, FrameLayout.LayoutParams.MATCH_PARENT, Gravity.CENTER))
            addView(form, FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT))
        })
        updateStatus()
    }

    override fun onResume() {
        super.onResume()
        resumed = true
        videoView.onResume()
        if (streaming) streamer.start(pcIp())
        updateReceiver()
        uiHandler.post(statusUpdater)
    }

    override fun onPause() {
        super.onPause()
        resumed = false
        updateReceiver()
        videoView.onPause()
        streamer.stop()
        if (adjustPanel.visibility == View.VISIBLE) saveLayout(adjustPanel.eyeLayout)
        uiHandler.removeCallbacks(statusUpdater)
    }

    override fun onDestroy() {
        super.onDestroy()
        decoderTarget?.release()
        decoderTarget = null
    }

    override fun onKeyDown(keyCode: Int, event: KeyEvent): Boolean {
        if (streaming && (keyCode == KeyEvent.KEYCODE_VOLUME_UP || keyCode == KeyEvent.KEYCODE_VOLUME_DOWN)) {
            if (event.repeatCount == 0) streamer.recenter()
            return true
        }
        return super.onKeyDown(keyCode, event)
    }

    @Deprecated("Back closes the adjustment or stops streaming instead of closing the app")
    override fun onBackPressed() {
        when {
            adjustPanel.visibility == View.VISIBLE -> setPanelOpen(false)
            streaming -> stopStreaming()
            else -> super.onBackPressed()
        }
    }

    // OpenGL makes a new decoder target each time it starts, which happens once the video view is
    // first shown. The previous target died with the old OpenGL context.
    private fun onDecoderTarget(target: DecoderTarget) {
        receiver?.stop()
        receiver = null
        decoderTarget?.release()
        decoderTarget = target
        updateReceiver()
    }

    // Video is received only while streaming, in the foreground, with somewhere to draw it.
    private fun updateReceiver() {
        val target = decoderTarget
        if (streaming && resumed && target != null) {
            if (receiver == null) receiver = VideoReceiver(target.surface, streamer::requestKeyframe).also { it.start() }
        } else {
            receiver?.stop()
            receiver = null
        }
    }

    private fun setPanelOpen(open: Boolean) {
        adjustPanel.visibility = if (open) View.VISIBLE else View.GONE
        if (!open) saveLayout(adjustPanel.eyeLayout)
        updateStatus()
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
        updateReceiver()
        updateStatus()
    }

    private fun stopStreaming() {
        streaming = false
        streamer.stop()
        updateReceiver()
        if (adjustPanel.visibility == View.VISIBLE) setPanelOpen(false)
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
        val error = streamer.lastError ?: receiver?.lastError ?: renderer.lastError
        if (!streaming) {
            statusText.setIfChanged(
                if (error != null) "Error: $error" else "Escribe la IP de la PC y pulsa Iniciar.\n" +
                    "Con el video en marcha, toca la pantalla para ajustar cada ojo a tu visor.",
            )
            return
        }
        // The overlay comes back whenever the video stops, with counters that show where it stops.
        val r = receiver
        val playing = r != null && SystemClock.elapsedRealtime() - r.lastFrameShownMs < 1000
        val adjusting = adjustPanel.visibility == View.VISIBLE
        val overlayVisibility = if ((!playing || error != null) && !adjusting) View.VISIBLE else View.GONE
        if (videoStatus.visibility != overlayVisibility) videoStatus.visibility = overlayVisibility
        if (overlayVisibility == View.GONE) return
        val video = if (r == null) "" else "\nVideo: ${r.packetsReceived.get()} paquetes, " +
            "${r.framesComplete.get()} fotogramas (${r.keyframes.get()} clave, ${r.framesLost.get()} perdidos), " +
            "${r.framesShown.get()} mostrados" + (r.decoderName?.let { " con $it" } ?: "")
        videoStatus.setIfChanged(
            (if (error != null) "Error: $error\n" else "") +
                "Enviando sensores a ${pcIp()} (${streamer.packetsSent.get()} paquetes)" + video +
                "\nEsperando el video de SteamVR…\nAtrás: detener · Volumen: recentrar · Tocar: ajustar el visor",
        )
    }

    // Avoids redrawing (and relayout) every tick when nothing changed.
    private fun TextView.setIfChanged(value: String) {
        if (text.toString() != value) text = value
    }

    private fun pcIp() = ipField.text.toString().trim()

    private fun prefs() = getSharedPreferences("settings", MODE_PRIVATE)

    // The whole screen in landscape, including the parts under the system bars and the camera hole.
    @Suppress("DEPRECATION")
    private fun screenMetrics(): DisplayMetrics {
        val metrics = DisplayMetrics()
        windowManager.defaultDisplay.getRealMetrics(metrics)
        if (metrics.heightPixels > metrics.widthPixels) {
            metrics.widthPixels = metrics.heightPixels.also { metrics.heightPixels = metrics.widthPixels }
        }
        return metrics
    }

    // Some phones report a made-up physical density; then the rounded one is closer.
    private fun pixelsPerMm(metrics: DisplayMetrics): Float {
        val dpi = (metrics.xdpi + metrics.ydpi) / 2f
        return (if (dpi in 100f..1000f) dpi else metrics.densityDpi.toFloat()) / 25.4f
    }

    private fun loadLayout(defaults: EyeLayout): EyeLayout {
        val p = prefs()
        return EyeLayout(
            widthMm = p.getFloat(KEY_EYE_WIDTH, defaults.widthMm),
            heightMm = p.getFloat(KEY_EYE_HEIGHT, defaults.heightMm),
            separationMm = p.getFloat(KEY_EYE_SEPARATION, defaults.separationMm),
            offsetMm = p.getFloat(KEY_EYE_OFFSET, defaults.offsetMm),
            k1 = p.getFloat(KEY_LENS_K1, defaults.k1),
            k2 = p.getFloat(KEY_LENS_K2, defaults.k2),
        )
    }

    private fun saveLayout(layout: EyeLayout) {
        prefs().edit()
            .putFloat(KEY_EYE_WIDTH, layout.widthMm)
            .putFloat(KEY_EYE_HEIGHT, layout.heightMm)
            .putFloat(KEY_EYE_SEPARATION, layout.separationMm)
            .putFloat(KEY_EYE_OFFSET, layout.offsetMm)
            .putFloat(KEY_LENS_K1, layout.k1)
            .putFloat(KEY_LENS_K2, layout.k2)
            .apply()
    }

    private companion object {
        const val KEY_PC_IP = "pc_ip"
        const val KEY_EYE_WIDTH = "eye_width_mm"
        const val KEY_EYE_HEIGHT = "eye_height_mm"
        const val KEY_EYE_SEPARATION = "eye_separation_mm"
        const val KEY_EYE_OFFSET = "eye_offset_mm"
        const val KEY_LENS_K1 = "lens_k1"
        const val KEY_LENS_K2 = "lens_k2"
    }
}
