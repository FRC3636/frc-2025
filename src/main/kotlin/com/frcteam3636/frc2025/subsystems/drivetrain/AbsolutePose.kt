package com.frcteam3636.frc2025.subsystems.drivetrain

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
//import org.photonvision.PhotonCamera
//import org.photonvision.PhotonPoseEstimator
import java.nio.ByteBuffer
import kotlin.math.pow

interface AbsolutePoseIO {
    class Inputs : LoggableInputs {
        // The most recent measurement from the pose estimator.
        var measurement: AbsolutePoseMeasurement? = null

        override fun toLog(table: LogTable) {
//            if (measurement != null) {
//                table.put("Measurement", measurement)
//            }
        }

        override fun fromLog(table: LogTable?) {
//            measurement = table?.get("Measurement", measurement)!![0]
        }
    }

    fun updateInputs(inputs: Inputs)

    val cameraConnected: Boolean
}

class LimelightPoseIOReal(private val name: String) : AbsolutePoseIO {
    private val table = NetworkTableInstance
        .getDefault()
        .getTable(name)
    private val stddev = VecBuilder.fill(.7, .7, 9999999.0)

    private val botPose = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(null)
    private val cl = table.getDoubleTopic("cl").subscribe(0.0)
    private val tl = table.getDoubleTopic("tl").subscribe(0.0)
    private val disconnectTimeout = Timer().apply {
        start()
    }

    override fun updateInputs(inputs: AbsolutePoseIO.Inputs) {
        inputs.measurement = botPose.readQueue().lastOrNull()?.let { update ->

            val x = update.value[0]
            val y = update.value[1]
            val z = update.value[2]
            val roll = Units.degreesToRadians(update.value[3])
            val pitch = Units.degreesToRadians(update.value[4])
            val yaw = Units.degreesToRadians(update.value[5])
            val tagCount = update.value[7]

            disconnectTimeout.restart()

            if (tagCount == 0.0 || Drivetrain.gyroRate > DegreesPerSecond.of(720.0)) {
                return
            }

            val latency = cl.get() + tl.get()

            AbsolutePoseMeasurement(
                pose = Pose3d(Translation3d(x, y, z), Rotation3d(roll, pitch, yaw)),
                timestamp = (update.timestamp * 1e-6) - (latency * 1e-3),
                stdDeviation = stddev
            ).also {
                Logger.recordOutput("Absolute Pose/$name/Pose", it.pose)
                Logger.recordOutput("Absolute Pose/$name/Tag Count", tagCount)
            }
        }
    }

    override val cameraConnected
        get() = disconnectTimeout.hasElapsed(1.0)
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

data class AbsolutePoseMeasurement(val pose: Pose3d, val timestamp: Double, val stdDeviation: Matrix<N3, N1>) :
    StructSerializable {
    companion object {
        @JvmField
        @Suppress("unused")
        val struct = AbsolutePoseMeasurementStruct()
    }
}

fun SwerveDrivePoseEstimator.addAbsolutePoseMeasurement(measurement: AbsolutePoseMeasurement) {
    addVisionMeasurement(
        measurement.pose.toPose2d(),
        measurement.timestamp,
//        measurement.stdDeviation // seems to fire the bot into orbit
    )
}

class AbsolutePoseMeasurementStruct : Struct<AbsolutePoseMeasurement> {
    override fun getTypeClass(): Class<AbsolutePoseMeasurement> = AbsolutePoseMeasurement::class.java
    override fun getTypeName(): String {
        return "struct:AbsolutePoseMeasurement"
    }

    override fun getTypeString(): String = "struct:AbsolutePoseMeasurement"
    override fun getSize(): Int = Pose3d.struct.size + Struct.kSizeDouble + 3 * Struct.kSizeDouble
    override fun getSchema(): String = "Pose3d pose; double timestamp; double stdDeviation[3];"
    override fun unpack(bb: ByteBuffer): AbsolutePoseMeasurement =
        AbsolutePoseMeasurement(
            pose = Pose3d.struct.unpack(bb),
            timestamp = bb.double,
            stdDeviation = VecBuilder.fill(bb.double, bb.double, bb.double)
        )

    override fun pack(bb: ByteBuffer, value: AbsolutePoseMeasurement) {
        Pose3d.struct.pack(bb, value.pose)
        bb.putDouble(value.timestamp)
        bb.putDouble(value.stdDeviation[0, 0])
        bb.putDouble(value.stdDeviation[1, 0])
        bb.putDouble(value.stdDeviation[2, 0])
    }
}

internal val APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)

@Suppress("unused")
internal const val APRIL_TAG_AMBIGUITY_FILTER = 0.3
internal val APRIL_TAG_STD_DEV = { distance: Double, count: Int ->
    val distanceMultiplier = (distance - (count - 1) * 3).pow(2.0)
    val translationalStdDev = (0.05 / count) * distanceMultiplier + 0.0
    val rotationalStdDev = 0.2 * distanceMultiplier + 0.1
    VecBuilder.fill(
        translationalStdDev, translationalStdDev, rotationalStdDev
    )
}
