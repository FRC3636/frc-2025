plugins {
    id("org.jetbrains.kotlin.jvm").version("2.1.0")
    id("edu.wpi.first.GradleRIO").version("2025.1.1")
}

group = "org.team9432.lib"

dependencies {
    wpi.java.deps.wpilib().forEach { implementation(it) }
    wpi.java.vendor.java().forEach { implementation(it) }

    wpi.java.deps.wpilibJniDebug(wpi.platforms.desktop).forEach { nativeDebug(it) }
    wpi.java.vendor.jniDebug(wpi.platforms.desktop).forEach { nativeDebug(it) }
    wpi.sim.enableDebug().forEach { simulationDebug(it) }

    wpi.java.deps.wpilibJniRelease(wpi.platforms.desktop).forEach { nativeRelease(it) }
    wpi.java.vendor.jniRelease(wpi.platforms.desktop).forEach { nativeRelease(it) }

    wpi.java.deps.wpilibJniRelease(wpi.platforms.desktop).forEach { nativeRelease(it) }
    wpi.java.vendor.jniRelease(wpi.platforms.desktop).forEach { nativeRelease(it) }
    wpi.sim.enableRelease().forEach { simulationRelease(it) }

    wpi.sim.enableRelease().forEach { simulationRelease(it) }

    // Misc.
    implementation("com.squareup:kotlinpoet:1.14.2")
    implementation("com.squareup:kotlinpoet-ksp:1.14.2")
    implementation("com.google.devtools.ksp:symbol-processing-api:2.1.0-1.0.29")
}

test {
    useJUnitPlatform()
    systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
}

repositories {
    mavenCentral()
    maven {
        url = uri("https://maven.pkg.github.com/Mechanical-Advantage/AdvantageKit")
        credentials {
            username = "Mechanical-Advantage-Bot"
            password = "\u0067\u0068\u0070\u005f\u006e\u0056\u0051\u006a\u0055\u004f\u004c\u0061\u0079\u0066\u006e\u0078\u006e\u0037\u0051\u0049\u0054\u0042\u0032\u004c\u004a\u006d\u0055\u0070\u0073\u0031\u006d\u0037\u004c\u005a\u0030\u0076\u0062\u0070\u0063\u0051"
        }
    }
}

// Set to true to use debug for JNI.
wpi.java.debugJni = false

sourceSets.main {
    java.srcDirs("src/main/kotlin")
}

wpi.java.configureTestTasks(test)