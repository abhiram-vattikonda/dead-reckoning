// Top-level build file.
plugins {
    // NOTE (AGP 9+): Kotlin Android support is built into AGP — do NOT apply
    // 'org.jetbrains.kotlin.android' anymore. Only the Compose compiler plugin
    // below is still required.
    id("com.android.application") version "9.3.2" apply false
    id("org.jetbrains.kotlin.plugin.compose") version "2.3.20" apply false
}
