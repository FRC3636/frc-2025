package com.frcteam3636.frc2025.subsystems.drivetrain

//import org.photonvision.PhotonCamera
//import org.photonvision.PhotonPoseEstimator
import com.frcteam3636.frc2025.utils.LimelightHelpers
import com.frcteam3636.frc2025.utils.QuestNav
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Time
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team9432.annotation.Logged
import java.nio.ByteBuffer

class AbsolutePoseProviderInputs : LoggableInputs {
    /**
     * The most recent measurement from the pose estimator.
     */
    var measurement: AbsolutePoseMeasurement? = null

    /**
     * If there is little to no ambiguity.
     */
    var isHighQualityReading = false

    /**
     * Whether the provider is connected.
     */
    var connected = false

    override fun toLog(table: LogTable) {
        if (measurement != null) {
            table.put("Measurement", measurement)
        }
        table.put("Connected", connected)
    }

    override fun fromLog(table: LogTable) {
        measurement = table.get("Measurement", measurement)[0]
        connected = table.get("Connected", connected)
    }
}


interface AbsolutePoseProvider {
    fun updateInputs(inputs: AbsolutePoseProviderInputs)
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
        inputs.isHighQualityReading = hasReading && readingIsNearby

        // We assume the camera has disconnected if there's no new updates.
        inputs.connected = inputs.measurement?.let {
            val timeSinceLastUpdate = Seconds.of(Timer.getTimestamp()) - it.timestamp
            timeSinceLastUpdate > Seconds.of(0.25)
        } ?: false
    }
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

class QuestNavLocalizer {
    private val questNav = QuestNav()
    private val lowBatteryAlert = Alert("The Meta Quest battery is below 20%!", AlertType.kWarning)

    fun resetPose(pose: Pose2d) {
        questNav.resetPosition(pose)
    }

    fun updateInputs(inputs: QuestNavInputs) {
        questNav.finalizeCommands()
        lowBatteryAlert.set(questNav.batteryPercent < 20.0)

        inputs.connected = questNav.connected
        inputs.pose = questNav.pose
    }
}

//@Suppress("unused")
//class PhotonVisionPoseIOReal(name: String, chassisToCamera: Transform3d) : AbsolutePoseIO {
//    private val camera = PhotonCamera(name).apply { driverMode = false }
//    private val estimator =
//        PhotonPoseEstimator(
//            APRIL_TAG_FIELD_LAYOUT,
//            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//            camera,
//            chassisToCamera
//        )
//
//    override fun updateInputs(inputs: AbsolutePoseIO.Inputs) {
//        inputs.measurement = null
//        estimator.update().ifPresent {
//            inputs.measurement = AbsolutePoseMeasurement(
//                it.estimatedPose,
//                it.timestampSeconds,
//                APRIL_TAG_STD_DEV(it.estimatedPose.translation.norm, it.targetsUsed.size)
//            )
//        }
//    }
//
//    override val cameraConnected
//        get() = camera.isConnected
//}

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

//internal val APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)

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