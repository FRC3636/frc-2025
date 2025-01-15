package com.frcteam3636.frc2025.subsystems.drivetrain

//import org.photonvision.PhotonCamera
//import org.photonvision.PhotonPoseEstimator
import com.frcteam3636.frc2025.utils.LimelightHelpers
import com.frcteam3636.frc2025.utils.QuestNav
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Time
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.team9432.annotation.Logged
import java.nio.ByteBuffer

class AbsolutePoseProviderInputs : LoggableInputs {
    /**
     * The most recent measurement from the pose estimator.
     */
    var measurement: AbsolutePoseMeasurement? = null

    /**
     * Whether the provider is connected.
     */
    var connected = false

    var observedTags: IntArray = intArrayOf()

    override fun toLog(table: LogTable) {
        if (measurement != null) {
            table.put("Measurement", measurement)
        }
        table.put("Connected", connected)
        table.put("ObservedTags", observedTags)
    }

    override fun fromLog(table: LogTable) {
        measurement = table.get("Measurement", measurement)[0]
        connected = table.get("Connected", connected)
        observedTags = table.get("ObservedTags", observedTags)
    }
}


interface AbsolutePoseProvider {
    fun updateInputs(inputs: AbsolutePoseProviderInputs)

    /**
     * If there is little to no ambiguity.
     */
    val hasHighQualityReading: Boolean
}

class LimelightPoseProvider(private val name: String) : AbsolutePoseProvider {
    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        // This is mostly pulled from the LimeLight docs.
        // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#4-field-localization-with-megatag
        val estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name)
        val trackedTag = LimelightHelpers.getTargetPose3d_RobotSpace(name)

        // Only trust the measurement if we see multiple tags.
        inputs.measurement = if (estimate != null && estimate.tagCount >= 2) {
            AbsolutePoseMeasurement(
                estimate.pose,
                Seconds.of(estimate.timestampSeconds),
                VecBuilder.fill(0.7, 0.7, 9999999.0),
            )
        } else {
            null
        }

        val hasReading = (estimate?.tagCount ?: 0) >= 1
        val readingIsNearby = trackedTag.translation.norm < DISTANT_APRIL_TAG_DISTANCE.`in`(Meters)
        hasHighQualityReading = hasReading && readingIsNearby

        // We assume the camera has disconnected if there's no new updates.
        inputs.connected = inputs.measurement?.let {
            val timeSinceLastUpdate = Seconds.of(Timer.getTimestamp()) - it.timestamp
            timeSinceLastUpdate > Seconds.of(0.25)
        } == true
    }

    override var hasHighQualityReading = false
        private set
}

@Suppress("unused")
class CameraSimPoseProvider(name: String, val chassisToCamera: Transform3d) : AbsolutePoseProvider {
    private val camera = PhotonCamera(name)
    private val simProperties = SimCameraProperties().apply {
        setCalibration(1280, 800, Rotation2d(LIMELIGHT_FOV))
        fps = 120.0
        avgLatencyMs = 51.0
        latencyStdDevMs = 5.0
    }
    val sim = PhotonCameraSim(camera, simProperties)

    private val estimator =
        PhotonPoseEstimator(
            APRIL_TAGS,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            chassisToCamera
        )

    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        inputs.connected = true
        inputs.measurement = null
        val unreadResults = camera.allUnreadResults
        val latestResult = unreadResults.lastOrNull()
        if (latestResult != null) {
            estimator.update(latestResult).ifPresent {
                inputs.measurement = AbsolutePoseMeasurement(
                    it.estimatedPose.toPose2d(),
                    Seconds.of(it.timestampSeconds),
                    VecBuilder.fill(0.7, 0.7, 9999999.0)
                )
            }
            inputs.observedTags = latestResult.targets.map {
                it.fiducialId
            }.toIntArray()
        }
    }

    override val hasHighQualityReading = true
}

@Logged
open class QuestNavInputs {
    /**
     * The most recent measurement from the Quest.
     */
    var pose = Pose2d()

    /**
     * Whether the provider is connected.
     */
    var connected = false
}

class QuestNavLocalizer(
    /**
     * The location of the QuestNav device relative to the robot chassis.
     */
    deviceOffset: Transform2d,
) {
    private val questNav = QuestNav()
    private val lowBatteryAlert = Alert("The Meta Quest battery is below 40%!", AlertType.kWarning)
    private val deviceToChassis = deviceOffset.inverse()

    fun resetPose(pose: Pose2d) {
        questNav.resetPosition(pose)
    }

    fun updateInputs(inputs: QuestNavInputs) {
        questNav.finalizeCommands()
        lowBatteryAlert.set(questNav.batteryPercent < 40.0)

        inputs.connected = questNav.connected
        inputs.pose = questNav.pose.transformBy(deviceToChassis)
    }
}

data class AbsolutePoseMeasurement(
    val pose: Pose2d,
    val timestamp: Time,
    /**
     * Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in
     * radians). Increase these numbers to trust the vision pose measurement less.
     */
    val stdDeviation: Matrix<N3, N1>
) : StructSerializable {
    companion object {
        @JvmField
        @Suppress("unused")
        val struct = AbsolutePoseMeasurementStruct()
    }
}

fun SwerveDrivePoseEstimator.addAbsolutePoseMeasurement(measurement: AbsolutePoseMeasurement) {
    addVisionMeasurement(
        measurement.pose,
        measurement.timestamp.`in`(Seconds),
        measurement.stdDeviation // FIXME: seems to fire the bot into orbit...?
    )
}

class AbsolutePoseMeasurementStruct : Struct<AbsolutePoseMeasurement> {
    override fun getTypeClass(): Class<AbsolutePoseMeasurement> = AbsolutePoseMeasurement::class.java
    override fun getTypeName(): String {
        return "struct:AbsolutePoseMeasurement"
    }

    override fun getTypeString(): String = "struct:AbsolutePoseMeasurement"
    override fun getSize(): Int = Pose3d.struct.size + Struct.kSizeDouble + 3 * Struct.kSizeDouble
    override fun getSchema(): String = "Pose2d pose; double timestamp; double stdDeviation[3];"
    override fun unpack(bb: ByteBuffer): AbsolutePoseMeasurement =
        AbsolutePoseMeasurement(
            pose = Pose2d.struct.unpack(bb),
            timestamp = Seconds.of(bb.double),
            stdDeviation = VecBuilder.fill(bb.double, bb.double, bb.double)
        )

    override fun pack(bb: ByteBuffer, value: AbsolutePoseMeasurement) {
        Pose2d.struct.pack(bb, value.pose)
        bb.putDouble(value.timestamp.`in`(Seconds))
        bb.putDouble(value.stdDeviation[0, 0])
        bb.putDouble(value.stdDeviation[1, 0])
        bb.putDouble(value.stdDeviation[2, 0])
    }
}


//internal const val APRIL_TAG_AMBIGUITY_FILTER = 0.3
//internal val APRIL_TAG_STD_DEV = { distance: Double, count: Int ->
//    val distanceMultiplier = (distance - (count - 1) * 3).pow(2.0)
//    val translationalStdDev = (0.05 / count) * distanceMultiplier + 0.0
//    val rotationalStdDev = 0.2 * distanceMultiplier + 0.1
//    VecBuilder.fill(
//        translationalStdDev, translationalStdDev, rotationalStdDev
//    )
//}

val DISTANT_APRIL_TAG_DISTANCE = Meters.of(6.0)!!
val LIMELIGHT_FOV = Degrees.of(75.76079874010732)
