import edu.wpi.first.deployutils.deploy.artifact.FileTreeArtifact
import edu.wpi.first.gradlerio.deploy.roborio.FRCJavaArtifact
import edu.wpi.first.gradlerio.deploy.roborio.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import org.gradle.plugins.ide.idea.model.IdeaLanguageLevel
import org.jetbrains.kotlin.gradle.dsl.JvmTarget
import java.net.URI

plugins {
    java
    kotlin("jvm") version "2.1.0"
    id("edu.wpi.first.GradleRIO") version "2025.1.1"
    idea
    id("com.peterabeles.gversion") version "1.10.3"
    id("com.google.devtools.ksp")
}


val javaVersion: JavaVersion = JavaVersion.VERSION_17
val javaLanguageVersion: JavaLanguageVersion by extra { JavaLanguageVersion.of(javaVersion.toString()) }
val jvmVendor: JvmVendorSpec by extra { JvmVendorSpec.ADOPTIUM }
val kotlinJvmTarget = JvmTarget.fromTarget(javaVersion.toString())

@Suppress("PropertyName")
val ROBOT_MAIN_CLASS = "com.frcteam3636.frc2025.Main"

gversion {
    srcDir = layout.buildDirectory.dir("generated/gversion/main/kotlin").get().toString()
    language = "kotlin"
    classPackage = "com.frcteam3636.version"
    dateFormat = "yyyy-MM-dd HH:mm:ss z"
    timeZone = "America/Los_Angeles" // Use preferred time zone
    indent = "    "
}

// Define my targets (RoboRIO) and artifacts (deployable files)
// This is added by GradleRIO's backing project DeployUtils.
deploy {
    targets {
        create("roborio", RoboRIO::class.java) {
            // Team number is loaded either from the .wpilib/wpilib_preferences.json
            // or from command line. If not found an exception will be thrown.
            // You can use project.frc.getTeamOrDefault(####) instead of project.frc.teamNumber
            // if you want to store a team number in this file.
            team = project.frc.teamNumber
            debug = project.frc.getDebugOrDefault(false)
            artifacts.create("frcJava", FRCJavaArtifact::class.java) {

            }
            artifacts.create("frcStaticFileDeploy", FileTreeArtifact::class.java) {
                files = project.fileTree("src/main/deploy")
                directory = "/home/lvuser/deploy"
                // Change to true to delete files on roboRIO that no longer exist in deploy directory of this project
                deleteOldFiles = false
            }
        }
    }
}
val deployArtifact = deploy.targets.getByName("roborio").artifacts.getByName("frcJava") as FRCJavaArtifact

// Set to true to use debug for JNI.
wpi.java.debugJni = false

// Set this to true to enable desktop support.
val includeDesktopSupport = true

repositories {
    // Set repositories to use. In Gradle DSL builds, the GradleRIO plugin automatically configures these repos.
    // But with a Kotlin DSL build file, they are not getting automatically configured.
    // If anyone can determine a way to apply programmatically, please open a ticket at
    // https://gitlab.com/Javaru/frc-intellij-idea-plugin/-/issues and I will update the template.
    maven {
        name = "WPILocal"
        url = wpi.frcHome.map { it.dir("maven") }.get().asFile.toURI()
    }
    maven {
        name = "WPIOfficialRelease"
        url = URI("https://frcmaven.wpi.edu/artifactory/release")
    }
    maven {
        name = "WPIFRCMavenVendorCacheRelease"
        url = URI("https://frcmaven.wpi.edu/artifactory/vendor-mvn-release")
    }
    wpi.vendor.vendorRepos.forEach {
        maven {
            name = it.name
            url = URI(it.url)
        }
    }
    mavenCentral()
}

dependencies {
    annotationProcessor(wpi.java.deps.wpilibAnnotations())
    implementation(wpi.java.deps.wpilib())
    implementation(wpi.java.vendor.java())

    roborioDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.roborio))
    roborioDebug(wpi.java.vendor.jniDebug(NativePlatforms.roborio))

    roborioRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.roborio))
    roborioRelease(wpi.java.vendor.jniRelease(NativePlatforms.roborio))

    nativeDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.desktop))
    nativeDebug(wpi.java.vendor.jniDebug(NativePlatforms.desktop))
    simulationDebug(wpi.sim.enableDebug())

    nativeRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.desktop))
    nativeRelease(wpi.java.vendor.jniRelease(NativePlatforms.desktop))
    simulationRelease(wpi.sim.enableRelease())

    testImplementation(platform("org.junit:junit-bom:5.11.4"))
    testImplementation("org.junit.jupiter:junit-jupiter-api")
    testImplementation("org.junit.jupiter:junit-jupiter-params")
    testRuntimeOnly("org.junit.jupiter:junit-jupiter-engine")

    implementation(project(":annotation"))
    ksp(project(":annotation"))
}

java {
    toolchain {
        languageVersion.set(javaLanguageVersion)
        vendor.set(jvmVendor)
    }
}

tasks.compileJava {
    options.encoding = Charsets.UTF_8.name()
    // Configure string concat to always inline compile
    options.compilerArgs.add("-XDstringConcat=inline")
}
    
kotlin {
    compilerOptions {
        jvmTarget.set(kotlinJvmTarget)
    }

    jvmToolchain {
        // https://kotlinlang.org/docs/gradle-configure-project.html#gradle-java-toolchains-support
        languageVersion.set(javaLanguageVersion)
        vendor.set(jvmVendor)
    }
}

tasks.test {
    useJUnitPlatform()
    systemProperty("junit.jupiter.extensions.autodetection.enabled", "true")
}

// Simulation configuration (e.g. environment variables).
wpi.sim.addGui().defaultEnabled = true
wpi.sim.addDriverstation()


// Setting up my Jar File. In this case, adding all libraries into the main jar ('fat jar')
// in order to make them all available at runtime. Also adding the manifest so WPILib
// knows where to look for our Robot Class.
tasks.jar {
    from(project.configurations.runtimeClasspath.get().map { if (it.isDirectory) it else zipTree(it) })
    from(sourceSets.main.get().allSource)
    manifest(edu.wpi.first.gradlerio.GradleRIOPlugin.javaManifest(ROBOT_MAIN_CLASS))
    duplicatesStrategy = DuplicatesStrategy.INCLUDE
}

tasks {
    register("replayWatch", type = JavaExec::class) {
        mainClass = "org.littletonrobotics.junction.ReplayWatch"
        classpath = sourceSets.main.get().runtimeClasspath
    }
}

// Configure jar and deploy tasks
deployArtifact.setJarTask(tasks.jar.get())
wpi.java.configureExecutableTasks(tasks.jar.get())
wpi.java.configureTestTasks(tasks.test.get())


idea {
    project {
        // The project.sourceCompatibility setting is not always picked up, so we set explicitly
        languageLevel = IdeaLanguageLevel(javaVersion)
    }
    module {
        // Improve development & (especially) debugging experience (and IDEA's capabilities) by having libraries' source & javadoc attached
        isDownloadJavadoc = true
        isDownloadSources = true
        // Exclude the .vscode directory from indexing and search
        excludeDirs.add(file(".run" ))
        excludeDirs.add(file(".vscode" ))
    }
}

// Helper Functions to keep syntax cleaner
// @formatter:off
fun DependencyHandler.addDependencies(configurationName: String, dependencies: List<Provider<String>>) = dependencies.forEach { add(configurationName, it) }
fun DependencyHandler.roborioDebug(dependencies: List<Provider<String>>) = addDependencies("roborioDebug", dependencies)
fun DependencyHandler.roborioRelease(dependencies: List<Provider<String>>) = addDependencies("roborioRelease", dependencies)
fun DependencyHandler.nativeDebug(dependencies: List<Provider<String>>) = addDependencies("nativeDebug", dependencies)
fun DependencyHandler.simulationDebug(dependencies: List<Provider<String>>) = addDependencies("simulationDebug", dependencies)
fun DependencyHandler.nativeRelease(dependencies: List<Provider<String>>) = addDependencies("nativeRelease", dependencies)
fun DependencyHandler.simulationRelease(dependencies: List<Provider<String>>) = addDependencies("simulationRelease", dependencies)
fun DependencyHandler.implementation(dependencies: List<Provider<String>>) = dependencies.forEach{ implementation(it) }
fun DependencyHandler.annotationProcessor(dependencies: List<Provider<String>>) = dependencies.forEach{ annotationProcessor(it) }
 // @formatter:on