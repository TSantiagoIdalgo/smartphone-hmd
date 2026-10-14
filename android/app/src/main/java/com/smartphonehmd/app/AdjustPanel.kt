package com.smartphonehmd.app

import android.content.Context
import android.graphics.Color
import android.view.Gravity
import android.widget.Button
import android.widget.CheckBox
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.SeekBar
import android.widget.TextView
import kotlin.math.roundToInt

/**
 * Sliders that size and move both eyes and set the lens correction. Every change goes to
 * [onChange] at once, so the video behind the panel shows it live. [defaults] is the unadjusted
 * full-screen layout; the size sliders go up to twice its values. [onGrid] turns on a grid that
 * stays after the panel closes, to check the lens correction from inside the headset. [onPredict]
 * turns the PC's head-movement prediction on or off, next to the measured latency it is based on.
 */
class AdjustPanel(
    context: Context,
    private val defaults: EyeLayout,
    initial: EyeLayout,
    predict: Boolean,
    private val onChange: (EyeLayout) -> Unit,
    onGrid: (Boolean) -> Unit,
    onPredict: (Boolean) -> Unit,
    onDone: () -> Unit,
) : ScrollView(context) {
    var eyeLayout = initial
        private set

    private class Slider(
        val label: String,
        val min: Float,
        val max: Float,
        val step: Float,
        val decimals: Int,
        val unit: String,
        val get: (EyeLayout) -> Float,
        val set: (EyeLayout, Float) -> EyeLayout,
    )

    private val sliders = listOf(
        Slider("Ancho de cada ojo", 20f, defaults.widthMm * 2f, 0.5f, 1, " mm", { it.widthMm }, { l, v -> l.copy(widthMm = v) }),
        Slider("Alto de cada ojo", 20f, defaults.heightMm * 2f, 0.5f, 1, " mm", { it.heightMm }, { l, v -> l.copy(heightMm = v) }),
        Slider("Separación entre centros", 30f, defaults.separationMm * 2f, 0.5f, 1, " mm", { it.separationMm }, { l, v -> l.copy(separationMm = v) }),
        Slider("Altura de los centros", -defaults.heightMm / 2f, defaults.heightMm / 2f, 0.5f, 1, " mm", { it.offsetMm }, { l, v -> l.copy(offsetMm = v) }),
        Slider("Corrección de lente k1", 0f, 1f, 0.01f, 2, "", { it.k1 }, { l, v -> l.copy(k1 = v) }),
        Slider("Corrección de lente k2", 0f, 1f, 0.01f, 2, "", { it.k2 }, { l, v -> l.copy(k2 = v) }),
    )
    private val seekBars = mutableListOf<SeekBar>()
    private val labels = mutableListOf<TextView>()
    private val latencyText = text("")

    init {
        setBackgroundColor(0xB0000000.toInt())
        val column = LinearLayout(context).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(32, 24, 32, 24)
        }
        column.addView(text("Ajusta con el móvil fuera del visor, pulsa Listo y compruébalo dentro."))
        column.addView(CheckBox(context).apply {
            text = "Cuadrícula: sus líneas deben verse rectas a través de la lente"
            setTextColor(Color.WHITE)
            setOnCheckedChangeListener { _, checked -> onGrid(checked) }
        })
        column.addView(latencyText)
        column.addView(CheckBox(context).apply {
            text = "Anticipar el movimiento de la cabeza (compensa la latencia)"
            setTextColor(Color.WHITE)
            isChecked = predict
            setOnCheckedChangeListener { _, checked -> onPredict(checked) }
        })
        for (slider in sliders) {
            val label = text("")
            val seekBar = SeekBar(context).apply {
                max = ((slider.max - slider.min) / slider.step).roundToInt()
                setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
                    override fun onProgressChanged(bar: SeekBar, progress: Int, fromUser: Boolean) {
                        if (!fromUser) return
                        eyeLayout = slider.set(eyeLayout, slider.min + progress * slider.step)
                        showValue(slider, label)
                        onChange(eyeLayout)
                    }

                    override fun onStartTrackingTouch(bar: SeekBar) {}
                    override fun onStopTrackingTouch(bar: SeekBar) {}
                })
            }
            labels += label
            seekBars += seekBar
            column.addView(label)
            column.addView(seekBar)
        }
        column.addView(LinearLayout(context).apply {
            gravity = Gravity.CENTER
            addView(Button(context).apply {
                text = "Restablecer"
                setOnClickListener {
                    eyeLayout = defaults
                    showAll()
                    onChange(eyeLayout)
                }
            })
            addView(Button(context).apply {
                text = "Listo"
                setOnClickListener { onDone() }
            })
        })
        addView(column)
        showAll()
        showLatency(null)
    }

    /** [ms]: from sending the sensors to showing the frame drawn with them; null if not measured yet. */
    fun showLatency(ms: Float?) {
        val value = if (ms == null) "midiendo…" else "${ms.roundToInt()} ms"
        val text = "Latencia medida (movimiento a imagen): $value"
        if (latencyText.text.toString() != text) latencyText.text = text
    }

    private fun showAll() {
        sliders.forEachIndexed { i, slider ->
            seekBars[i].progress = ((slider.get(eyeLayout) - slider.min) / slider.step).roundToInt()
            showValue(slider, labels[i])
        }
    }

    private fun showValue(slider: Slider, label: TextView) {
        label.text = "${slider.label}: ${"%.${slider.decimals}f".format(slider.get(eyeLayout))}${slider.unit}"
    }

    private fun text(value: String) = TextView(context).apply {
        text = value
        setTextColor(Color.WHITE)
        textSize = 14f
    }
}
