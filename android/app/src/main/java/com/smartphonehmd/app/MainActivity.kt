package com.smartphonehmd.app

import android.app.Activity
import android.content.Context
import android.graphics.Color
import android.hardware.SensorManager
import android.net.wifi.WifiManager
import android.opengl.GLSurfaceView
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.text.Editable
import android.text.InputType
import android.text.TextWatcher
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
import java.util.concurrent.Executors

/**
 * Press Start: the phone finds the PC on its own (or uses the IP typed in), sends its sensors and
 * shows the PC's video full screen, each eye where the headset's lenses need it. If the video
 * stops, it looks for the PC again and follows it to a new address. Either volume button
 * recenters the view, which works with the phone already inside the headset. A tap opens the eye
 * adjustment; Back closes it or stops streaming.
 */
class MainActivity : Activity() {
    private val meter = LatencyMeter()
    private val timewarp = Timewarp()
    private lateinit var streamer: SensorStreamer
    private lateinit var finder: PcFinder
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
    // Android 10 makes the caller wait up to seconds while it takes or drops a Wi-Fi lock, so
    // that happens on its own thread. wifiLock is only touched there.
    private val wifiExecutor = Executors.newSingleThreadExecutor()
    private var wifiLock: WifiManager.WifiLock? = null

    private var streaming = false
    private var resumed = false
    private var streamStartMs = 0L
    // When each address last answered the PC finder. A PC on USB and Wi-Fi answers on both.
    private val lastAnswerMs = HashMap<String, Long>()
    // An IP typed in this session wins over the one the finder reports; a saved one does not.
    private var ipTypedByUser = false
    private var settingIp = false
    private val uiHandler = Handler(Looper.getMainLooper())
    private val statusUpdater = object : Runnable {
        override fun run() {
            updateStatus()
            updateFinder()
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
        preferFastestRefreshRate()
        streamer = SensorStreamer(getSystemService(Context.SENSOR_SERVICE) as SensorManager, meter, timewarp)
        streamer.predict = prefs().getBoolean(KEY_PREDICT, true)
        timewarp.enabled = prefs().getBoolean(KEY_TIMEWARP, true)
        finder = PcFinder(::onPcFound)

        ipField = EditText(this).apply {
            hint = "IP de la PC (se busca sola)"
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            setSingleLine()
            setText(prefs().getString(KEY_PC_IP, ""))
            addTextChangedListener(object : TextWatcher {
                override fun afterTextChanged(s: Editable?) {
                    if (!settingIp) ipTypedByUser = true
                }

                override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
                override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
            })
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
        renderer = EyeRenderer(videoView, pxPerMm, layout, timewarp, ::onDecoderTarget)
        videoView.setRenderer(renderer)
        videoView.renderMode = GLSurfaceView.RENDERMODE_WHEN_DIRTY
        adjustPanel = AdjustPanel(
            this, defaults, layout, streamer.predict, timewarp.enabled,
            onChange = { renderer.layout = it },
            onGrid = { renderer.showGrid = it },
            onPredict = {
                streamer.predict = it
                prefs().edit().putBoolean(KEY_PREDICT, it).apply()
            },
            onTimewarp = {
                timewarp.enabled = it
                prefs().edit().putBoolean(KEY_TIMEWARP, it).apply()
            },
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
        if (streaming) streamer.start(pcIp().ifEmpty { null })
        updateReceiver()
        updateWifiLock()
        uiHandler.post(statusUpdater)
    }

    override fun onPause() {
        super.onPause()
        resumed = false
        updateReceiver()
        updateWifiLock()
        finder.stop()
        videoView.onPause()
        streamer.stop()
        if (adjustPanel.visibility == View.VISIBLE) saveLayout(adjustPanel.eyeLayout)
        uiHandler.removeCallbacks(statusUpdater)
    }

    override fun onDestroy() {
        super.onDestroy()
        decoderTarget?.release()
        decoderTarget = null
        wifiExecutor.shutdown()
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
            if (receiver == null) {
                receiver = VideoReceiver(target.surface, meter, timewarp, streamer::requestKeyframe).also { it.start() }
            }
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

    // With no IP yet, streaming starts anyway and the PC finder supplies one.
    private fun startStreaming() {
        val ip = pcIp()
        if (ip.isNotEmpty()) prefs().edit().putString(KEY_PC_IP, ip).apply()
        streaming = true
        streamStartMs = SystemClock.elapsedRealtime()
        meter.reset()
        streamer.start(ip.ifEmpty { null })
        form.visibility = View.GONE
        videoView.visibility = View.VISIBLE
        videoStatus.visibility = View.VISIBLE
        enterFullScreen()
        updateReceiver()
        updateWifiLock()
        updateStatus()
    }

    private fun stopStreaming() {
        streaming = false
        streamer.stop()
        updateReceiver()
        updateWifiLock()
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

    private fun videoPlaying(): Boolean {
        val r = receiver ?: return false
        return SystemClock.elapsedRealtime() - r.lastFrameShownMs < 1000
    }

    // Looks for the PC while there is none to talk to: on the start screen, and while streaming
    // gets no video (the PC may have changed address).
    private fun updateFinder() {
        if (resumed && (!streaming || !videoPlaying())) finder.start() else finder.stop()
    }

    private fun answering(ip: String?): Boolean {
        val at = lastAnswerMs[ip ?: return false] ?: return false
        return SystemClock.elapsedRealtime() - at < RECONNECT_AFTER_MS
    }

    // Keeps the address in use while the PC still answers there, and moves to another one only
    // when it stops: the cable was unplugged, or the router gave the PC a new IP.
    private fun onPcFound(ip: String) {
        lastAnswerMs[ip] = SystemClock.elapsedRealtime()
        if (!streaming) {
            if (!ipTypedByUser && pcIp() != ip && !answering(pcIp())) setIp(ip)
            updateStatus()
            return
        }
        val target = streamer.targetHost
        val sinceVideoMs = SystemClock.elapsedRealtime() - maxOf(streamStartMs, receiver?.lastFrameShownMs ?: 0L)
        if (target == null || (target != ip && !answering(target) && sinceVideoMs > RECONNECT_AFTER_MS)) {
            setIp(ip)
            prefs().edit().putString(KEY_PC_IP, ip).apply()
            streamer.setTarget(ip)
            streamStartMs = SystemClock.elapsedRealtime() // give the new address time to answer
        }
    }

    private fun setIp(ip: String) {
        settingIp = true
        ipField.setText(ip)
        settingIp = false
    }

    // Keeps Wi-Fi out of power saving, which delays and drops packets, while streaming.
    private fun updateWifiLock() {
        val hold = streaming && resumed
        wifiExecutor.execute {
            try {
                val lock = wifiLock ?: run {
                    val wifi = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
                    @Suppress("DEPRECATION")
                    val mode = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                        WifiManager.WIFI_MODE_FULL_LOW_LATENCY
                    } else {
                        WifiManager.WIFI_MODE_FULL_HIGH_PERF
                    }
                    wifi.createWifiLock(mode, "smartphone-hmd").apply { setReferenceCounted(false) }.also { wifiLock = it }
                }
                if (hold) {
                    if (!lock.isHeld) lock.acquire()
                } else if (lock.isHeld) {
                    lock.release()
                }
            } catch (_: Exception) {
                // Only a performance hint.
            }
        }
    }

    private fun updateStatus() {
        val error = streamer.lastError ?: receiver?.lastError ?: renderer.lastError
        if (!streaming) {
            statusText.setIfChanged(
                when {
                    error != null -> "Error: $error"
                    answering(pcIp()) -> "PC encontrada en ${pcIp()}. Pulsa Iniciar."
                    else -> "Buscando la PC… Abre SteamVR en ella, o escribe su IP."
                } + "\nCon el video en marcha, toca la pantalla para ajustar cada ojo a tu visor.",
            )
            return
        }
        if (adjustPanel.visibility == View.VISIBLE) adjustPanel.showLatency(meter.averageMs())
        // The overlay comes back whenever the video stops, with counters that show where it stops.
        val r = receiver
        val playing = videoPlaying()
        val adjusting = adjustPanel.visibility == View.VISIBLE
        val overlayVisibility = if ((!playing || error != null) && !adjusting) View.VISIBLE else View.GONE
        if (videoStatus.visibility != overlayVisibility) videoStatus.visibility = overlayVisibility
        if (overlayVisibility == View.GONE) return
        val video = if (r == null) "" else "\nVideo: ${r.packetsReceived.get()} paquetes, " +
            "${r.framesComplete.get()} fotogramas (${r.keyframes.get()} clave, ${r.framesLost.get()} perdidos), " +
            "${r.framesShown.get()} mostrados" + (r.decoderName?.let { " con $it" } ?: "")
        val target = streamer.targetHost
        val sending = if (target == null) {
            "Buscando la PC… Abre SteamVR en ella."
        } else {
            "Enviando sensores a $target (${streamer.packetsSent.get()} paquetes)"
        }
        videoStatus.setIfChanged(
            (if (error != null) "Error: $error\n" else "") + sending + video +
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
    // Each decoded frame waits for the screen's next refresh, so a 90 or 120 Hz screen shows it
    // sooner. Phones often run apps at 60 Hz unless they ask.
    private fun preferFastestRefreshRate() {
        val display = windowManager.defaultDisplay
        val current = display.mode
        val fastest = display.supportedModes
            .filter { it.physicalWidth == current.physicalWidth && it.physicalHeight == current.physicalHeight }
            .maxByOrNull { it.refreshRate } ?: return
        window.attributes = window.attributes.apply { preferredDisplayModeId = fastest.modeId }
    }

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
        const val KEY_PREDICT = "predict"
        const val KEY_TIMEWARP = "timewarp"
        const val RECONNECT_AFTER_MS = 3000L
    }
}
