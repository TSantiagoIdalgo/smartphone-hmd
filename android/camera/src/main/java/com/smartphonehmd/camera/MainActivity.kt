package com.smartphonehmd.camera

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.Color
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.TotalCaptureResult
import android.hardware.display.DisplayManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.text.Editable
import android.text.InputType
import android.text.TextWatcher
import android.util.Log
import android.util.Range
import android.util.Size
import android.view.Gravity
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.TextView
import androidx.activity.ComponentActivity
import androidx.activity.result.contract.ActivityResultContracts
import androidx.annotation.OptIn
import androidx.camera.camera2.interop.Camera2CameraControl
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.Camera2Interop
import androidx.camera.camera2.interop.CaptureRequestOptions
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.Camera
import androidx.camera.core.CameraSelector
import androidx.camera.core.FocusMeteringAction
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.core.SurfaceOrientedMeteringPointFactory
import androidx.camera.core.resolutionselector.AspectRatioStrategy
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import java.util.concurrent.Executor
import java.util.concurrent.Executors
import kotlin.random.Random

/**
 * A body camera: shows what the camera sees and sends it as H.264 video to the PC, which it finds
 * on its own (or uses the IP typed in). The PC (tools/body_tracker.py) finds the person's pose in
 * it and sends the points back, drawn here over the picture. Mount the phone, stand where you play,
 * check the hip, knee and ankle points are green, then fix the exposure so the picture stops
 * changing. A tap on the picture hides the controls.
 */
class MainActivity : ComponentActivity() {
    private val uiHandler = Handler(Looper.getMainLooper())
    private val mainThread = Executor { uiHandler.post(it) }
    // Camera frames and everything that feeds the encoder run on this one thread, in order.
    private val cameraThread = Executors.newSingleThreadExecutor()
    private lateinit var link: PcLink
    private lateinit var previewView: PreviewView
    private lateinit var poseOverlay: PoseOverlay
    private lateinit var panel: View
    private lateinit var statsText: TextView
    private lateinit var lockButton: Button
    private lateinit var fpsButton: Button
    private lateinit var cameraButton: Button
    private var deviceId = 0

    private var cameraProvider: ProcessCameraProvider? = null
    private var camera: Camera? = null
    private var preview: Preview? = null
    private var analysis: ImageAnalysis? = null
    private var fpsChoices = emptyList<Int>()
    @Volatile private var currentFps = 0
    private var usingFront = false
    private var cameraStatus = "Abriendo la cámara…"
    private var cameraError: String? = null

    // Written on the UI thread or the camera's, read on the camera thread or for the screen.
    @Volatile private var exposureLocked = false
    @Volatile private var timestampIsRealtime = false
    private val cameraRate = RateMeter()
    private val videoRate = RateMeter()
    private val bitRate = RateMeter()
    private val poseRate = RateMeter()
    // Where the time goes before a frame leaves: the camera (from the picture to this app) and
    // the sending of its packets; the encoder measures its own.
    private val cameraDelay = AverageMeter()
    private val sendDelay = AverageMeter()

    // Camera thread only, except the @Volatile ones, shown on screen or used by the link.
    @Volatile private var encoder: VideoEncoder? = null
    private var encoderRetryNs = 0L
    @Volatile private var videoStatus = "Esperando la cámara…"
    @Volatile private var imageSize = ""

    // The poses that come back from the PC, written on the link's thread.
    @Volatile private var pcInferenceMs = 0f
    @Volatile private var roundTripMs = 0f
    @Volatile private var personFound = false
    @Volatile private var hipVisibility = 0f
    @Volatile private var kneeVisibility = 0f
    @Volatile private var ankleVisibility = 0f

    private val askCamera = registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
        if (granted) {
            startCamera()
        } else {
            cameraStatus = "Sin permiso de cámara: dáselo en Ajustes > Apps > HMD Cámara"
        }
    }

    private val statusUpdater = object : Runnable {
        override fun run() {
            updateStats()
            uiHandler.postDelayed(this, 500)
        }
    }

    // Half a turn keeps the activity, so the image's rotation is updated here; a quarter turn
    // recreates it, which builds everything again with the new rotation.
    private val displayListener = object : DisplayManager.DisplayListener {
        override fun onDisplayChanged(displayId: Int) {
            val rotation = displayRotation()
            preview?.targetRotation = rotation
            analysis?.targetRotation = rotation
        }

        override fun onDisplayAdded(displayId: Int) {}
        override fun onDisplayRemoved(displayId: Int) {}
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        deviceId = loadDeviceId()
        usingFront = prefs().getBoolean(KEY_FRONT, false)
        link = PcLink(deviceId)
        link.manualIp = prefs().getString(KEY_PC_IP, "")
        link.onPose = ::showPose
        link.onKeyframeRequest = { encoder?.requestKeyframe() }
        buildViews()
        if (intent.getBooleanExtra(EXTRA_SELF_TEST, false)) cameraThread.execute { EncoderSelfTest.run(this) }
        if (checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            startCamera()
        } else {
            askCamera.launch(Manifest.permission.CAMERA)
        }
        (getSystemService(Context.DISPLAY_SERVICE) as DisplayManager).registerDisplayListener(displayListener, uiHandler)
    }

    override fun onResume() {
        super.onResume()
        link.start()
        uiHandler.post(statusUpdater)
    }

    override fun onPause() {
        super.onPause()
        link.stop()
        uiHandler.removeCallbacks(statusUpdater)
    }

    override fun onDestroy() {
        super.onDestroy()
        (getSystemService(Context.DISPLAY_SERVICE) as DisplayManager).unregisterDisplayListener(displayListener)
        analysis?.clearAnalyzer()
        cameraThread.execute { closeEncoder() }
        cameraThread.shutdown()
    }

    private fun buildViews() {
        previewView = PreviewView(this).apply { scaleType = PreviewView.ScaleType.FIT_CENTER }
        poseOverlay = PoseOverlay(this).apply {
            setOnClickListener { panel.visibility = if (panel.visibility == View.VISIBLE) View.GONE else View.VISIBLE }
        }
        statsText = TextView(this).apply {
            setTextColor(Color.WHITE)
            textSize = 14f
        }
        val ipField = EditText(this).apply {
            hint = "IP de la PC (se busca sola)"
            setTextColor(Color.WHITE)
            setHintTextColor(Color.LTGRAY)
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            setSingleLine()
            setText(link.manualIp)
            addTextChangedListener(object : TextWatcher {
                override fun afterTextChanged(s: Editable?) {
                    val ip = s?.toString().orEmpty().trim()
                    link.manualIp = ip
                    prefs().edit().putString(KEY_PC_IP, ip).apply()
                }

                override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
                override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
            })
        }
        lockButton = button("Fijar exposición") { setExposureLocked(!exposureLocked) }
        fpsButton = button("fps") { nextFps() }
        cameraButton = button("") { switchCamera() }
        updateButtons()
        val buttons = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            for (b in listOf(lockButton, fpsButton, cameraButton)) {
                addView(b, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
            }
        }
        panel = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(Color.argb(170, 0, 0, 0))
            setPadding(24, 16, 24, 16)
            addView(statsText)
            addView(ipField)
            addView(buttons)
        }
        // The controls go at the top: the feet, which matter most, are at the bottom of the picture.
        setContentView(FrameLayout(this).apply {
            setBackgroundColor(Color.BLACK)
            addView(previewView, FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT))
            addView(poseOverlay, FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT))
            addView(panel, FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.WRAP_CONTENT, Gravity.TOP))
        })
    }

    private fun button(label: String, onClick: () -> Unit) = Button(this).apply {
        text = label
        textSize = 12f
        isAllCaps = false
        setOnClickListener { onClick() }
    }

    private fun updateButtons() {
        lockButton.text = if (exposureLocked) "Soltar exposición" else "Fijar exposición"
        lockButton.isEnabled = camera != null
        fpsButton.text = if (currentFps > 0) "$currentFps fps" else "fps"
        fpsButton.isEnabled = camera != null && fpsChoices.size > 1
        cameraButton.text = if (usingFront) "Frontal" else "Trasera"
        cameraButton.isEnabled = camera != null
    }

    private fun updateStats() {
        val now = SystemClock.elapsedRealtimeNanos()
        statsText.text = buildString {
            append("Cámara %04X · ".format(deviceId and 0xFFFF))
            val error = cameraError
            when {
                error != null -> append(error)
                camera == null -> append(cameraStatus)
                else -> append("${if (usingFront) "frontal" else "trasera"} $imageSize · ${"%.1f".format(cameraRate.perSecond(now))} cuadros/s")
            }
            append('\n')
            append(videoStatus)
            val frames = videoRate.perSecond(now)
            if (frames > 0) {
                append(" · ${"%.1f".format(frames)} cuadros/s · ${"%.1f".format(bitRate.perSecond(now) * 8 / 1e6f)} Mb/s")
            }
            append('\n')
            val delays = listOf("cámara" to cameraDelay, "codificador" to encoder?.delay, "envío" to sendDelay)
                .mapNotNull { (stage, meter) -> meter?.average(now)?.let { "$stage ${"%.0f".format(it)} ms" } }
            if (delays.isNotEmpty()) {
                append("Demora: ${delays.joinToString(" · ")}")
                append('\n')
            }
            val poses = poseRate.perSecond(now)
            if (poses > 0) {
                append("Pose de la PC: ${"%.1f".format(poses)}/s · modelo ${"%.0f".format(pcInferenceMs)} ms · ")
                append("ida y vuelta ${"%.0f".format(roundTripMs)} ms")
                append('\n')
                if (personFound) {
                    append("Cadera ${percent(hipVisibility)} · rodillas ${percent(kneeVisibility)} · tobillos ${percent(ankleVisibility)}")
                } else {
                    append("No se ve a nadie")
                }
            } else {
                append("Sin poses de la PC: abre body_tracker.py en la PC")
            }
            append('\n')
            val target = link.target
            if (target != null) {
                append("PC $target · ${link.sentPackets} paquetes enviados")
            } else {
                append("Buscando la PC… (abre body_tracker.py)")
            }
        }
    }

    private fun percent(visibility: Float) = "${(visibility * 100).toInt()}%"

    private fun startCamera() {
        val future = ProcessCameraProvider.getInstance(this)
        future.addListener({
            cameraProvider = try {
                future.get()
            } catch (e: Exception) {
                Log.e(TAG, "Camera provider failed", e)
                cameraStatus = "No se pudo abrir la cámara: ${e.message}"
                return@addListener
            }
            bindCamera()
        }, mainThread)
    }

    @OptIn(ExperimentalCamera2Interop::class)
    private fun bindCamera() {
        val provider = cameraProvider ?: return
        provider.unbindAll()
        camera?.cameraInfo?.cameraState?.removeObservers(this)
        camera = null
        preview = null
        analysis = null
        exposureLocked = false
        cameraError = null
        val wanted = if (usingFront) CameraSelector.DEFAULT_FRONT_CAMERA else CameraSelector.DEFAULT_BACK_CAMERA
        val other = if (usingFront) CameraSelector.DEFAULT_BACK_CAMERA else CameraSelector.DEFAULT_FRONT_CAMERA
        val selector = listOf(wanted, other).firstOrNull { hasCamera(provider, it) }
        val info = selector?.filter(provider.availableCameraInfos)?.firstOrNull()
        if (selector == null || info == null) {
            Log.i(TAG, "No camera")
            cameraStatus = "Sin cámara"
            updateButtons()
            return
        }
        usingFront = selector == CameraSelector.DEFAULT_FRONT_CAMERA
        poseOverlay.mirrored = usingFront

        val characteristics = Camera2CameraInfo.from(info)
        // Realtime means the frame times are already on elapsedRealtime, the clock the PC syncs to.
        timestampIsRealtime = characteristics.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) ==
            CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME
        fpsChoices = characteristics.getCameraCharacteristic(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
            ?.filter { it.lower == it.upper && it.upper in 15..60 }
            ?.map { it.upper }
            ?.distinct()
            ?.sorted()
            .orEmpty()
        val savedFps = prefs().getInt(KEY_FPS, 30)
        currentFps = fpsChoices.lastOrNull { it <= savedFps } ?: fpsChoices.firstOrNull() ?: 0

        val rotation = displayRotation()
        val fourByThree = ResolutionSelector.Builder()
            .setAspectRatioStrategy(AspectRatioStrategy.RATIO_4_3_FALLBACK_AUTO_STRATEGY)
        val previewBuilder = Preview.Builder()
            .setResolutionSelector(fourByThree.build())
            .setTargetRotation(rotation)
        // 640x480 is plenty: the pose model looks at 256x256. The frames come as the sensor sees
        // them, in YUV, ready for the encoder; the PC turns them upright.
        val analysisBuilder = ImageAnalysis.Builder()
            .setResolutionSelector(
                fourByThree
                    .setResolutionStrategy(ResolutionStrategy(Size(640, 480), ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER))
                    .build(),
            )
            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
            .setTargetRotation(rotation)
        // Counts every frame the camera takes, analysed or not.
        Camera2Interop.Extender(analysisBuilder).setSessionCaptureCallback(object : CameraCaptureSession.CaptureCallback() {
            override fun onCaptureCompleted(session: CameraCaptureSession, request: CaptureRequest, result: TotalCaptureResult) {
                cameraRate.tick(SystemClock.elapsedRealtimeNanos())
            }
        })
        if (currentFps > 0) {
            // A fixed rate, so the camera does not slow down to brighten a dim room.
            val range = Range(currentFps, currentFps)
            Camera2Interop.Extender(previewBuilder).setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, range)
            Camera2Interop.Extender(analysisBuilder).setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, range)
        }

        val newPreview = previewBuilder.build().also { it.setSurfaceProvider(previewView.surfaceProvider) }
        val newAnalysis = analysisBuilder.build().also { it.setAnalyzer(cameraThread) { image -> analyze(image) } }
        try {
            camera = provider.bindToLifecycle(this, selector, newPreview, newAnalysis)
        } catch (e: Exception) {
            Log.e(TAG, "Camera bind failed", e)
            cameraStatus = "No se pudo abrir la cámara: ${e.message}"
            updateButtons()
            return
        }
        preview = newPreview
        analysis = newAnalysis
        camera?.cameraInfo?.cameraState?.observe(this) { state ->
            cameraError = state.error?.let { "Error de cámara ${it.code}${if (currentFps > 30) ": prueba con menos fps" else ""}" }
        }
        Log.i(TAG, "Camera ${if (usingFront) "front" else "back"}, $currentFps fps of $fpsChoices, realtime clock $timestampIsRealtime")
        updateButtons()
    }

    private fun hasCamera(provider: ProcessCameraProvider, selector: CameraSelector): Boolean = try {
        provider.hasCamera(selector)
    } catch (_: Exception) {
        false
    }

    @OptIn(ExperimentalCamera2Interop::class)
    private fun setExposureLocked(locked: Boolean) {
        val camera = camera ?: return
        exposureLocked = locked
        Camera2CameraControl.from(camera.cameraControl).setCaptureRequestOptions(
            CaptureRequestOptions.Builder()
                .setCaptureRequestOption(CaptureRequest.CONTROL_AE_LOCK, locked)
                .setCaptureRequestOption(CaptureRequest.CONTROL_AWB_LOCK, locked)
                .build(),
        )
        if (locked) {
            // Focus once on the middle of the picture and stay there.
            val middle = SurfaceOrientedMeteringPointFactory(1f, 1f).createPoint(0.5f, 0.5f)
            camera.cameraControl.startFocusAndMetering(
                FocusMeteringAction.Builder(middle, FocusMeteringAction.FLAG_AF).disableAutoCancel().build(),
            )
        } else {
            camera.cameraControl.cancelFocusAndMetering()
        }
        updateButtons()
    }

    private fun nextFps() {
        if (fpsChoices.size < 2) return
        val next = fpsChoices.firstOrNull { it > currentFps } ?: fpsChoices.first()
        prefs().edit().putInt(KEY_FPS, next).apply()
        bindCamera()
    }

    private fun switchCamera() {
        usingFront = !usingFront
        prefs().edit().putBoolean(KEY_FRONT, usingFront).apply()
        bindCamera()
    }

    // Camera thread: one camera frame in, to the encoder, which sends it once encoded.
    private fun analyze(image: ImageProxy) {
        try {
            val now = SystemClock.elapsedRealtimeNanos()
            val captureNs = captureTime(image.imageInfo.timestamp, now)
            cameraDelay.add(now, (now - captureNs) / 1e6f)
            imageSize = "${image.width}×${image.height}"
            val fps = if (currentFps > 0) currentFps else 30
            val current = encoder?.takeIf { it.width == image.width && it.height == image.height && it.fps == fps }
                ?: openEncoder(image.width, image.height, fps, now)
                ?: return
            val planes = Array(3) { YuvPlane(image.planes[it].buffer, image.planes[it].rowStride, image.planes[it].pixelStride) }
            val flags = if (exposureLocked) BodyProtocol.FLAG_EXPOSURE_LOCKED else 0
            current.encode(planes, VideoEncoder.Tag(captureNs, image.imageInfo.rotationDegrees / 90, flags))
        } catch (e: Exception) {
            Log.e(TAG, "Video encoding failed", e)
            videoStatus = "El codificador de video falló: ${e.message}"
            closeEncoder()
            encoderRetryNs = SystemClock.elapsedRealtimeNanos() + RETRY_NS
        } finally {
            image.close()
        }
    }

    // Camera thread. After a failure, waits a few seconds before trying the encoder again.
    private fun openEncoder(width: Int, height: Int, fps: Int, nowNs: Long): VideoEncoder? {
        closeEncoder()
        if (nowNs < encoderRetryNs) return null
        return try {
            VideoEncoder(width, height, fps, fps * BITS_PER_FRAME) { tag, keyframe, data, offset, length ->
                val startNs = SystemClock.elapsedRealtimeNanos()
                if (link.sendVideo(tag, width, height, keyframe, data, offset, length)) {
                    val sentNs = SystemClock.elapsedRealtimeNanos()
                    sendDelay.add(sentNs, (sentNs - startNs) / 1e6f)
                    videoRate.tick(sentNs)
                    bitRate.tick(sentNs, length.toLong())
                }
            }.also {
                encoder = it
                videoStatus = "Video H.264 (${it.name}${if (it.bitrateMode.isEmpty()) "" else ", ${it.bitrateMode}"})"
                Log.i(TAG, "Video encoder ${it.name} ${width}x$height at $fps fps")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Video encoder failed", e)
            videoStatus = "No se pudo abrir el codificador de video: ${e.message}"
            encoderRetryNs = nowNs + RETRY_NS
            null
        }
    }

    // Camera thread.
    private fun closeEncoder() {
        encoder?.close()
        encoder = null
    }

    // Link thread: a pose from the PC, drawn over the picture.
    private fun showPose(pose: BodyProtocol.Frame) {
        val now = SystemClock.elapsedRealtimeNanos()
        poseRate.tick(now)
        pcInferenceMs = pose.inferenceMs
        roundTripMs = (now - pose.captureNs) / 1e6f
        val count = if (pose.flags and BodyProtocol.FLAG_POSE != 0) pose.landmarkCount else 0
        personFound = count > 28
        if (personFound) {
            hipVisibility = pairVisibility(pose, 23)
            kneeVisibility = pairVisibility(pose, 25)
            ankleVisibility = pairVisibility(pose, 27)
        }
        poseOverlay.show(pose.landmarks, count, pose.width, pose.height)
    }

    /** The lower visibility of a left point and its right twin, which follows it. */
    private fun pairVisibility(pose: BodyProtocol.Frame, left: Int): Float =
        minOf(pose.landmarks[left * 5 + 3], pose.landmarks[(left + 1) * 5 + 3])

    // The camera's frame time on the elapsedRealtime clock. Cameras that do not use that clock use
    // uptime, which only differs by the time the phone spent asleep.
    private fun captureTime(sensorNs: Long, nowNs: Long): Long {
        val ns = if (timestampIsRealtime) sensorNs else sensorNs + (nowNs - System.nanoTime())
        return if (ns > nowNs || nowNs - ns > 1_000_000_000L) nowNs else ns
    }

    @Suppress("DEPRECATION")
    private fun displayRotation(): Int = windowManager.defaultDisplay.rotation

    private fun loadDeviceId(): Int {
        val prefs = prefs()
        if (!prefs.contains(KEY_DEVICE_ID)) prefs.edit().putInt(KEY_DEVICE_ID, Random.nextInt(1, Int.MAX_VALUE)).apply()
        return prefs.getInt(KEY_DEVICE_ID, 0)
    }

    private fun prefs() = getSharedPreferences("camera", Context.MODE_PRIVATE)

    private companion object {
        const val TAG = "HmdCamera"
        /** For the CI test on the emulator, which has no camera: see [EncoderSelfTest]. */
        const val EXTRA_SELF_TEST = "selftest"
        // About 2 Mb/s at 30 fps: clean enough for the pose model, light on the Wi-Fi.
        const val BITS_PER_FRAME = 70_000
        const val RETRY_NS = 3_000_000_000L
        const val KEY_DEVICE_ID = "device_id"
        const val KEY_PC_IP = "pc_ip"
        const val KEY_FPS = "fps"
        const val KEY_FRONT = "front"
    }
}
