package com.smartphonehmd.app

import android.graphics.SurfaceTexture
import android.opengl.GLES11Ext
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.Surface
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.atomic.AtomicInteger
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

/** The video decoder draws into [surface]; [texture] passes those frames to OpenGL. */
class DecoderTarget(val texture: SurfaceTexture, val surface: Surface) {
    fun release() {
        surface.release()
        texture.release()
    }
}

/**
 * Draws the side-by-side video with each eye in its own rectangle from [layout], pre-bent so the
 * headset's lens straightens it. Whenever OpenGL (re)starts it makes a new [DecoderTarget] and
 * hands it to [onTarget] on the main thread.
 */
class EyeRenderer(
    private val view: GLSurfaceView,
    private val pxPerMm: Float,
    layout: EyeLayout,
    private val onTarget: (DecoderTarget) -> Unit,
) : GLSurfaceView.Renderer {
    @Volatile var layout = layout
        set(value) {
            field = value
            view.requestRender()
        }

    /** Green lines that should look straight through the lens. */
    @Volatile var showGrid = false
        set(value) {
            field = value
            view.requestRender()
        }

    @Volatile var lastError: String? = null
        private set

    private val mainHandler = Handler(Looper.getMainLooper())
    private val pendingFrames = AtomicInteger(0)
    private var target: DecoderTarget? = null
    private var textureId = 0
    private val texMatrix = FloatArray(16)
    private val vertices = ByteBuffer.allocateDirect(16 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer()
    private var width = 0
    private var height = 0

    private var program = 0
    private var aPosition = 0
    private var aMm = 0
    private var uTexture = 0
    private var uTexMatrix = 0
    private var uHalfMm = 0
    private var uLens = 0
    private var uEye = 0
    private var uGrid = 0

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        program = try {
            buildProgram()
        } catch (e: RuntimeException) {
            lastError = "OpenGL: ${e.message}"
            Log.e(TAG, lastError!!)
            0
        }
        aPosition = GLES20.glGetAttribLocation(program, "aPosition")
        aMm = GLES20.glGetAttribLocation(program, "aMm")
        uTexture = GLES20.glGetUniformLocation(program, "uTexture")
        uTexMatrix = GLES20.glGetUniformLocation(program, "uTexMatrix")
        uHalfMm = GLES20.glGetUniformLocation(program, "uHalfMm")
        uLens = GLES20.glGetUniformLocation(program, "uLens")
        uEye = GLES20.glGetUniformLocation(program, "uEye")
        uGrid = GLES20.glGetUniformLocation(program, "uGrid")

        val ids = IntArray(1)
        GLES20.glGenTextures(1, ids, 0)
        textureId = ids[0]
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)

        val texture = SurfaceTexture(textureId)
        texture.setOnFrameAvailableListener {
            pendingFrames.incrementAndGet()
            view.requestRender()
        }
        val newTarget = DecoderTarget(texture, Surface(texture))
        pendingFrames.set(0)
        target = newTarget
        mainHandler.post { onTarget(newTarget) }
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        this.width = width
        this.height = height
        GLES20.glViewport(0, 0, width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        val texture = target?.texture
        // Take every frame that arrived since the last draw, so the newest one is shown.
        var frames = pendingFrames.getAndSet(0)
        try {
            while (frames-- > 0) texture?.updateTexImage()
        } catch (_: RuntimeException) {
            // The texture was released while switching targets; the new one takes over.
        }

        GLES20.glDisable(GLES20.GL_SCISSOR_TEST)
        GLES20.glClearColor(0f, 0f, 0f, 1f)
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT)
        if (program == 0 || texture == null || width == 0) return

        texture.getTransformMatrix(texMatrix)
        val l = layout
        GLES20.glUseProgram(program)
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId)
        GLES20.glUniform1i(uTexture, 0)
        GLES20.glUniformMatrix4fv(uTexMatrix, 1, false, texMatrix, 0)
        GLES20.glUniform2f(uHalfMm, l.widthMm / 2f, l.heightMm / 2f)
        GLES20.glUniform2f(uLens, l.k1, l.k2)
        GLES20.glUniform1f(uGrid, if (showGrid) 1f else 0f)

        // Each eye stays on its own half of the screen, like the headset's divider.
        GLES20.glEnable(GLES20.GL_SCISSOR_TEST)
        val half = width / 2
        GLES20.glScissor(0, 0, half, height)
        drawEye(0, l)
        GLES20.glScissor(half, 0, width - half, height)
        drawEye(1, l)
    }

    private fun drawEye(eye: Int, l: EyeLayout) {
        val r = l.eyeRect(eye, width, height, pxPerMm)
        val halfWidthMm = l.widthMm / 2f
        val halfHeightMm = l.heightMm / 2f
        val left = r.left / width * 2f - 1f
        val right = r.right / width * 2f - 1f
        val bottom = r.bottom / height * 2f - 1f
        val top = r.top / height * 2f - 1f
        // Per corner: position on screen, then millimetres from the eye's centre.
        vertices.clear()
        vertices.put(left).put(bottom).put(-halfWidthMm).put(-halfHeightMm)
        vertices.put(right).put(bottom).put(halfWidthMm).put(-halfHeightMm)
        vertices.put(left).put(top).put(-halfWidthMm).put(halfHeightMm)
        vertices.put(right).put(top).put(halfWidthMm).put(halfHeightMm)

        vertices.position(0)
        GLES20.glVertexAttribPointer(aPosition, 2, GLES20.GL_FLOAT, false, 16, vertices)
        GLES20.glEnableVertexAttribArray(aPosition)
        vertices.position(2)
        GLES20.glVertexAttribPointer(aMm, 2, GLES20.GL_FLOAT, false, 16, vertices)
        GLES20.glEnableVertexAttribArray(aMm)
        GLES20.glUniform1f(uEye, eye.toFloat())
        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)
    }

    private fun buildProgram(): Int {
        val vertexShader = compile(GLES20.GL_VERTEX_SHADER, VERTEX_SHADER)
        val fragmentShader = compile(GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER)
        val program = GLES20.glCreateProgram()
        GLES20.glAttachShader(program, vertexShader)
        GLES20.glAttachShader(program, fragmentShader)
        GLES20.glLinkProgram(program)
        val status = IntArray(1)
        GLES20.glGetProgramiv(program, GLES20.GL_LINK_STATUS, status, 0)
        if (status[0] == 0) {
            val log = GLES20.glGetProgramInfoLog(program)
            GLES20.glDeleteProgram(program)
            throw RuntimeException(log)
        }
        return program
    }

    private fun compile(type: Int, source: String): Int {
        val shader = GLES20.glCreateShader(type)
        GLES20.glShaderSource(shader, source)
        GLES20.glCompileShader(shader)
        val status = IntArray(1)
        GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, status, 0)
        if (status[0] == 0) {
            val log = GLES20.glGetShaderInfoLog(shader)
            GLES20.glDeleteShader(shader)
            throw RuntimeException(log)
        }
        return shader
    }

    private companion object {
        const val TAG = "SmartphoneHMD"

        val VERTEX_SHADER = """
            attribute vec2 aPosition;
            attribute vec2 aMm;
            varying vec2 vMm;
            void main() {
                vMm = aMm;
                gl_Position = vec4(aPosition, 0.0, 1.0);
            }
        """.trimIndent()

        // Barrel distortion that cancels the lens' pincushion: a point r from the lens centre shows
        // the image from r * (1 + k1 r² + k2 r⁴), with r in units of 30 mm (about a cheap lens'
        // radius) so k1 and k2 keep their effect when the eye is resized. Texture coordinates need
        // high precision: a 2400-pixel-wide video blurs with mediump.
        val FRAGMENT_SHADER = """
            #extension GL_OES_EGL_image_external : require
            #ifdef GL_FRAGMENT_PRECISION_HIGH
            precision highp float;
            #else
            precision mediump float;
            #endif
            uniform samplerExternalOES uTexture;
            uniform mat4 uTexMatrix;
            uniform vec2 uHalfMm;
            uniform vec2 uLens;
            uniform float uEye;
            uniform float uGrid;
            varying vec2 vMm;
            void main() {
                float r2 = dot(vMm, vMm) / 900.0;
                vec2 source = vMm * (1.0 + uLens.x * r2 + uLens.y * r2 * r2) / uHalfMm;
                if (abs(source.x) > 1.0 || abs(source.y) > 1.0) {
                    gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
                } else {
                    vec2 uv = vec2((uEye + 0.5 + 0.5 * source.x) * 0.5, 0.5 + 0.5 * source.y);
                    vec4 color = texture2D(uTexture, (uTexMatrix * vec4(uv, 0.0, 1.0)).xy);
                    vec2 line = abs(fract(source * 5.0 + 0.5) - 0.5);
                    if (uGrid > 0.5 && min(line.x, line.y) < 0.02) color = vec4(0.0, 1.0, 0.0, 1.0);
                    gl_FragColor = color;
                }
            }
        """.trimIndent()
    }
}
