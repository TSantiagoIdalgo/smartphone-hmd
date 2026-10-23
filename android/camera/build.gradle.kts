import java.net.URI
import java.security.MessageDigest
import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.smartphonehmd.camera"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.smartphonehmd.camera"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "0.1.0"
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    // MediaPipe maps the model straight out of the APK, which only works if it is stored uncompressed.
    androidResources {
        noCompress += listOf("task", "tflite")
    }
}

kotlin {
    compilerOptions {
        jvmTarget.set(JvmTarget.JVM_17)
    }
}

dependencies {
    val camerax = "1.4.1"
    implementation("androidx.camera:camera-core:$camerax")
    implementation("androidx.camera:camera-camera2:$camerax")
    implementation("androidx.camera:camera-lifecycle:$camerax")
    implementation("androidx.camera:camera-view:$camerax")
    implementation("androidx.activity:activity:1.9.3")
    implementation("com.google.mediapipe:tasks-vision:0.10.14")
    testImplementation("junit:junit:4.13.2")
}

/** Downloads MediaPipe's pose model once and checks it is the expected file. */
abstract class DownloadPoseModel : DefaultTask() {
    @get:Input
    abstract val url: Property<String>

    @get:Input
    abstract val sha256: Property<String>

    @get:OutputDirectory
    abstract val outputDir: DirectoryProperty

    @TaskAction
    fun download() {
        val bytes = URI(url.get()).toURL().openStream().use { it.readBytes() }
        val digest = MessageDigest.getInstance("SHA-256").digest(bytes).joinToString("") { "%02x".format(it) }
        check(digest == sha256.get()) { "Pose model checksum mismatch: $digest" }
        outputDir.get().file("pose_landmarker_lite.task").asFile.writeBytes(bytes)
    }
}

// One download task per variant: AGP points each one's output at that variant's generated assets.
androidComponents {
    onVariants { variant ->
        val download = tasks.register<DownloadPoseModel>("download${variant.name.replaceFirstChar { it.uppercase() }}PoseModel") {
            url.set("https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task")
            sha256.set("59929e1d1ee95287735ddd833b19cf4ac46d29bc7afddbbf6753c459690d574a")
        }
        variant.sources.assets?.addGeneratedSourceDirectory(download, DownloadPoseModel::outputDir)
    }
}
