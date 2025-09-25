package com.frcteam3636.frc2025.subsystems.drivetrain

//import org.photonvision.PhotonCamera
//import org.photonvision.PhotonPoseEstimator
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.utils.LimelightHelpers
import com.frcteam3636.frc2025.utils.math.degrees
import com.frcteam3636.frc2025.utils.math.inSeconds
import com.frcteam3636.frc2025.utils.math.meters
import com.frcteam3636.frc2025.utils.math.seconds
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import java.nio.ByteBuffer
import kotlin.concurrent.thread

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
}

/**
 * A Limelight localization algorithm.
 */
sealed class LimelightAlgorithm {
    /**
     * An older, less accurate localization algorithm.
     */
    object MegaTag : LimelightAlgorithm()

    /**
     * A newer and much more accurate algorithm that requires accurate gyro readings and a right-side-up Limelight.
     */
    class MegaTag2(private val gyroGetter: () -> Rotation2d, private val velocityGetter: () -> AngularVelocity) :
        LimelightAlgorithm() {
        val gyroPosition: Rotation2d
            get() = gyroGetter()
        val gyroVelocity: AngularVelocity
            get() = velocityGetter()
    }
}

data class LimelightMeasurement(
    var poseMeasurement: AbsolutePoseMeasurement? = null,
    var observedTags: IntArray = intArrayOf(),
) /* --- BEGIN KOTLIN COMPILER GENERATED CODE ---- */ {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as LimelightMeasurement

        if (poseMeasurement != other.poseMeasurement) return false
        if (!observedTags.contentEquals(other.observedTags)) return false

        return true
    }

    override fun hashCode(): Int {
        var result = poseMeasurement?.hashCode() ?: 0
        result = 31 * result + observedTags.contentHashCode()
        return result
    }
} /* --- END KOTLIN COMPILER GENERATED CODE ---- */

class LimelightPoseProvider(
    private val name: String,
    private val megaTagV2: LimelightAlgorithm.MegaTag2,
) : AbsolutePoseProvider {
    // References:
    // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation
    // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#4-field-localization-with-megatag

    private var observedTags = intArrayOf()

    private var measurement: AbsolutePoseMeasurement? = null
    private var mutex = Any()

    private var lastSeenHb: Double = 0.0
    private var hbSub = NetworkTableInstance.getDefault().getTable(name).getDoubleTopic("hb").subscribe(0.0)
    private var loopsSinceLastSeen: Int = 0

    private var currentAlgorithm: LimelightAlgorithm = LimelightAlgorithm.MegaTag

    init {
        thread(isDaemon = true) {
            while (true) {
                val temp = updateCurrentMeasurement()
                synchronized(mutex) {
                    measurement = temp.poseMeasurement
                    observedTags = temp.observedTags
                }
                Thread.sleep(Robot.period.toLong())
            }
        }
    }

    private fun updateCurrentMeasurement(): LimelightMeasurement {
        val measurement = LimelightMeasurement()

        if (!Robot.beforeFirstEnable && currentAlgorithm == LimelightAlgorithm.MegaTag)
            currentAlgorithm = megaTagV2

        when (val algorithm = currentAlgorithm) {
            is LimelightAlgorithm.MegaTag ->
                LimelightHelpers.getBotPoseEstimate_wpiBlue(name)?.let { estimate ->
                    measurement.observedTags = estimate.rawFiducials.mapNotNull { it?.id }.toIntArray()

                    // Reject zero tag or low-quality one tag readings
                    if (estimate.tagCount == 0) return measurement
                    if (estimate.tagCount == 1) {
                        val fiducial = estimate.rawFiducials[0]
                        if (fiducial == null
                            || fiducial.ambiguity > AMBIGUITY_THRESHOLD
                            || fiducial.distToCamera > MAX_SINGLE_TAG_DISTANCE
                        ) return measurement
                    }

                    measurement.poseMeasurement = AbsolutePoseMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds.seconds,
                        // This value is pulled directly from the Limelight docs (linked at the top of this class)
                        VecBuilder.fill(.5, .5, .25)
                    )
                }

            is LimelightAlgorithm.MegaTag2 -> {
                LimelightHelpers.SetRobotOrientation(
                    name,
                    algorithm.gyroPosition.degrees,
                    // The Limelight sample code leaves these as zero, and the API docs call them "Unnecessary."
                    0.0, 0.0, 0.0, 0.0, 0.0
                )

                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)?.let { estimate ->
                    measurement.observedTags = estimate.rawFiducials.mapNotNull { it?.id }.toIntArray()
                    val highSpeed = algorithm.gyroVelocity.abs(DegreesPerSecond) > 720.0
                    if (estimate.tagCount == 0 || highSpeed) return measurement

                    measurement.poseMeasurement = AbsolutePoseMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds.seconds,
//                        VecBuilder.fill(stdDevs[6], stdDevs[6], stdDevs[11]),
                        VecBuilder.fill(.5, .5, 9999999.0)
                    )
                }
            }

        }

        return measurement
    }

    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        synchronized(mutex) {
            inputs.measurement = measurement
            inputs.observedTags = observedTags

            // We assume the camera has disconnected if there are no new updates for several ticks.
            val hb = hbSub.get()
            inputs.connected = hb > lastSeenHb || loopsSinceLastSeen < CONNECTED_TIMEOUT
            if (hb == lastSeenHb)
                loopsSinceLastSeen++
            else
                loopsSinceLastSeen = 0
            lastSeenHb = hb
        }
    }

    companion object {
        /**
         * The acceptable distance for a single-April-Tag reading.
         *
         * This is a somewhat conservative limit, but it is only applied when using the old MegaTag v1 algorithm.
         * It's possible it could be increased if it's too restrictive.
         */
        private val MAX_SINGLE_TAG_DISTANCE = 3.meters

        /**
         * The acceptable ambiguity for a single-tag reading on MegaTag v1.
         */
        private const val AMBIGUITY_THRESHOLD = 0.7

        /**
         * The amount of time (in robot ticks) an update before considering the camera to be disconnected.
         */
        private const val CONNECTED_TIMEOUT = 250.0
    }
}

@Suppress("unused")
class CameraSimPoseProvider(name: String, val chassisToCamera: Transform3d) : AbsolutePoseProvider {
    private val camera = PhotonCamera(name)
    private val simProperties = SimCameraProperties().apply {
        setCalibration(1280, 800, Rotation2d(LIMELIGHT_FOV))
        fps = 20.0
        avgLatencyMs = 51.0
        latencyStdDevMs = 5.0
    }
    val sim = PhotonCameraSim(camera, simProperties)

    private val estimator =
        PhotonPoseEstimator(
            FIELD_LAYOUT,
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
                    it.timestampSeconds.seconds,
                    VecBuilder.fill(0.7, 0.7, 9999999.0)
                )
            }
            inputs.observedTags = latestResult.targets.map {
                it.fiducialId
            }.toIntArray()
        }
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
        measurement.timestamp.inSeconds(),
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
            timestamp = bb.double.seconds,
            stdDeviation = VecBuilder.fill(bb.double, bb.double, bb.double)
        )

    override fun pack(bb: ByteBuffer, value: AbsolutePoseMeasurement) {
        Pose2d.struct.pack(bb, value.pose)
        bb.putDouble(value.timestamp.inSeconds())
        bb.putDouble(value.stdDeviation[0, 0])
        bb.putDouble(value.stdDeviation[1, 0])
        bb.putDouble(value.stdDeviation[2, 0])
    }
}


//internal const val APRIL_TAG_AMBIGUITY_FILTER = 0.3
//internal val APRIL_TAG_STD_DEV = { distance: Double, count: Int ->
//    val distanceMultiplier = (distance - (count - 1) * 3).pow(2.0)
//    val translationalStdDev = (0.05 / count) * distanceMultiplier
//    val rotationalStdDev = 0.2 * distanceMultiplier + 0.1
//    VecBuilder.fill(
//        translationalStdDev, translationalStdDev, rotationalStdDev
//    )
//}

val LIMELIGHT_FOV = 75.76079874010732.degrees
