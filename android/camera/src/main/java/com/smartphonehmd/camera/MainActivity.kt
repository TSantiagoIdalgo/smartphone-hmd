package com.smartphonehmd.camera

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.Bitmap
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
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.core.Delegate
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.poselandmarker.PoseLandmarker
import java.util.concurrent.Executor
import java.util.concurrent.Executors
import kotlin.random.Random

/**
 * A body camera: shows what the camera sees, finds the person's pose on the phone with MediaPipe
 * and sends the points of every frame to the PC, which it finds on its own (or uses the IP typed
 * in). Mount the phone, stand where you play, check the hip, knee and ankle points are green, then
 * fix the exposure so the picture stops changing. A tap on the picture hides the controls.
 */
class MainActivity : ComponentActivity() {
    private val uiHandler = Handler(Looper.getMainLooper())
    private val mainThread = Executor { uiHandler.post(it) }
    // Frames and everything that touches the pose model run on this one thread, in order.
    private val poseThread = Executors.newSingleThreadExecutor()
    private lateinit var link: PcLink
    private lateinit var previewView: PreviewView
    private lateinit var overlay: PoseOverlay
    private lateinit var panel: View
    private lateinit var statsText: TextView
    private lateinit var lockButton: Button
    private lateinit var fpsButton: Button
    private lateinit var cameraButton: Button
    private lateinit var delegateButton: Button
    private var deviceId = 0

    private var cameraProvider: ProcessCameraProvider? = null
    private var camera: Camera? = null
    private var preview: Preview? = null
    private var analysis: ImageAnalysis? = null
    private var fpsChoices = emptyList<Int>()
    private var currentFps = 0
    private var usingFront = false
    private var cameraStatus = "Abriendo la cámara…"
    private var cameraError: String? = null

    // Written on the UI thread or the camera's, read on the pose thread or for the screen.
    @Volatile private var exposureLocked = false
    @Volatile private var timestampIsRealtime = false
    @Volatile private var useGpu = false
    private val cameraRate = RateMeter()
    private val poseRate = RateMeter()

    // Pose thread only, except the @Volatile results shown on screen.
    private var landmarker: PoseLandmarker? = null
    private var lastPoseMs = 0L
    private var bitmap: Bitmap? = null
    private val frame = BodyProtocol.Frame()
    @Volatile private var modelStatus = "Cargando el modelo…"
    @Volatile private var imageSize = ""
    @Volatile private var inferenceMs = 0f
    @Volatile private var delayMs = 0f
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
        useGpu = prefs().getBoolean(KEY_GPU, false)
        usingFront = prefs().getBoolean(KEY_FRONT, false)
        link = PcLink(deviceId)
        link.manualIp = prefs().getString(KEY_PC_IP, "")
        buildViews()
        poseThread.execute { openModel() }
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
        poseThread.execute {
            landmarker?.close()
            landmarker = null
        }
        poseThread.shutdown()
    }

    private fun buildViews() {
        previewView = PreviewView(this).apply { scaleType = PreviewView.ScaleType.FIT_CENTER }
        overlay = PoseOverlay(this).apply {
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
        delegateButton = button("") { switchDelegate() }
        updateButtons()
        val buttons = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            for (b in listOf(lockButton, fpsButton, cameraButton, delegateButton)) {
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
            addView(overlay, FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT))
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
        delegateButton.text = if (useGpu) "GPU" else "CPU"
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
            append(modelStatus)
            val poses = poseRate.perSecond(now)
            if (poses > 0) {
                append(" · ${"%.1f".format(poses)} poses/s · ${"%.0f".format(inferenceMs)} ms cada una · retraso ${"%.0f".format(delayMs)} ms")
                append('\n')
                if (personFound) {
                    append("Cadera ${percent(hipVisibility)} · rodillas ${percent(kneeVisibility)} · tobillos ${percent(ankleVisibility)}")
                } else {
                    append("No se ve a nadie")
                }
            }
            append('\n')
            val target = link.target
            if (target != null) {
                append("PC $target · ${link.sentPackets} paquetes enviados")
            } else {
                append("Buscando la PC… (abre body_receiver.py o SteamVR)")
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
        overlay.mirrored = usingFront

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
        // 640x480 is plenty: the model looks at 256x256. The image comes turned upright.
        val analysisBuilder = ImageAnalysis.Builder()
            .setResolutionSelector(
                fourByThree
                    .setResolutionStrategy(ResolutionStrategy(Size(640, 480), ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER))
                    .build(),
            )
            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
            .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
            .setOutputImageRotationEnabled(true)
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
        val newAnalysis = analysisBuilder.build().also { it.setAnalyzer(poseThread) { image -> analyze(image) } }
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

    private fun switchDelegate() {
        useGpu = !useGpu
        prefs().edit().putBoolean(KEY_GPU, useGpu).apply()
        updateButtons()
        poseThread.execute { openModel() }
    }

    // Pose thread.
    private fun openModel() {
        landmarker?.close()
        landmarker = null
        val gpu = useGpu
        val name = if (gpu) "GPU" else "CPU"
        modelStatus = "Cargando el modelo ($name)…"
        val options = PoseLandmarker.PoseLandmarkerOptions.builder()
            .setBaseOptions(
                BaseOptions.builder()
                    .setModelAssetPath(MODEL_FILE)
                    .setDelegate(if (gpu) Delegate.GPU else Delegate.CPU)
                    .build(),
            )
            .setRunningMode(RunningMode.VIDEO)
            .setNumPoses(1)
            .build()
        landmarker = try {
            PoseLandmarker.createFromOptions(this, options)
        } catch (e: Throwable) {
            Log.e(TAG, "Pose model failed to load on $name", e)
            modelStatus = "No se pudo cargar el modelo en $name${if (gpu) ", prueba CPU" else ""}: ${e.message}"
            return
        }
        Log.i(TAG, "Pose model ready on $name")
        modelStatus = "Modelo listo ($name)"
    }

    // Pose thread: one camera frame in, one packet out, with or without a person in it.
    private fun analyze(image: ImageProxy) {
        val captureNs = captureTime(image.imageInfo.timestamp, SystemClock.elapsedRealtimeNanos())
        val upright = try {
            toBitmap(image)
        } finally {
            image.close()
        }
        frame.seq++
        frame.captureNs = captureNs
        frame.deviceId = deviceId
        frame.width = upright.width
        frame.height = upright.height
        frame.flags = if (exposureLocked) BodyProtocol.FLAG_EXPOSURE_LOCKED else 0
        frame.landmarkCount = 0
        frame.inferenceMs = 0f
        imageSize = "${upright.width}×${upright.height}"

        val model = landmarker
        if (model != null) {
            // The model wants strictly increasing times in ms.
            val ms = maxOf(captureNs / 1_000_000, lastPoseMs + 1)
            lastPoseMs = ms
            val start = SystemClock.elapsedRealtimeNanos()
            val result = try {
                model.detectForVideo(BitmapImageBuilder(upright).build(), ms)
            } catch (e: Exception) {
                Log.e(TAG, "Pose detection failed", e)
                null
            }
            frame.inferenceMs = (SystemClock.elapsedRealtimeNanos() - start) / 1e6f
            val pose = result?.landmarks()?.firstOrNull()
            if (pose != null) {
                val count = minOf(pose.size, BodyProtocol.LANDMARK_COUNT)
                for (i in 0 until count) {
                    val point = pose[i]
                    val at = i * 5
                    frame.landmarks[at] = point.x()
                    frame.landmarks[at + 1] = point.y()
                    frame.landmarks[at + 2] = point.z()
                    frame.landmarks[at + 3] = point.visibility().orElse(0f)
                    frame.landmarks[at + 4] = point.presence().orElse(0f)
                }
                frame.landmarkCount = count
                frame.flags = frame.flags or BodyProtocol.FLAG_POSE
            }
            poseRate.tick(SystemClock.elapsedRealtimeNanos())
        }
        link.send(frame)

        inferenceMs = frame.inferenceMs
        delayMs = (SystemClock.elapsedRealtimeNanos() - captureNs) / 1e6f
        personFound = frame.landmarkCount > 0
        if (personFound) {
            hipVisibility = pairVisibility(23)
            kneeVisibility = pairVisibility(25)
            ankleVisibility = pairVisibility(27)
        }
        overlay.show(frame.landmarks, frame.landmarkCount, frame.width, frame.height)
    }

    /** The lower visibility of a left point and its right twin, which follows it. */
    private fun pairVisibility(left: Int): Float =
        minOf(frame.landmarks[left * 5 + 3], frame.landmarks[(left + 1) * 5 + 3])

    // The camera's frame time on the elapsedRealtime clock. Cameras that do not use that clock use
    // uptime, which only differs by the time the phone spent asleep.
    private fun captureTime(sensorNs: Long, nowNs: Long): Long {
        val ns = if (timestampIsRealtime) sensorNs else sensorNs + (nowNs - System.nanoTime())
        return if (ns > nowNs || nowNs - ns > 1_000_000_000L) nowNs else ns
    }

    // Copies the RGBA frame into a bitmap reused from frame to frame, unless its rows are padded.
    private fun toBitmap(image: ImageProxy): Bitmap {
        val plane = image.planes[0]
        if (plane.pixelStride != 4 || plane.rowStride != image.width * 4) return image.toBitmap()
        val target = bitmap?.takeIf { it.width == image.width && it.height == image.height }
            ?: Bitmap.createBitmap(image.width, image.height, Bitmap.Config.ARGB_8888).also { bitmap = it }
        plane.buffer.rewind()
        target.copyPixelsFromBuffer(plane.buffer)
        return target
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
        const val MODEL_FILE = "pose_landmarker_lite.task"
        const val KEY_DEVICE_ID = "device_id"
        const val KEY_PC_IP = "pc_ip"
        const val KEY_FPS = "fps"
        const val KEY_FRONT = "front"
        const val KEY_GPU = "gpu"
    }
}
