package com.frcteam3636.frc2025.utils

import com.frcteam3636.frc2025.utils.math.seconds
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.DoubleSubscriber
import edu.wpi.first.networktables.FloatArraySubscriber
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer

enum class QuestCommand(val id: Long) {
    /**
     * A value that represents the lack of a command.
     */
    None(0),

    /**
     * Recenters the player's view by adjusting the VR camera root transform to match the reset transform.
     * Similar to the effect of long-pressing the Oculus button.
     */
    HeadingRecenter(1),

    /**
     * Initiates a pose reset based on received NetworkTables data
     */
    PoseReset(2),

    /**
     * Responds with [PING_RESPONSE].
     */
    Ping(3)
}

const val COMMAND_NEEDS_FINALIZATION = 99L
const val PING_RESPONSE = 97L

class QuestNav {
    // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
    var nt4Instance: NetworkTableInstance = NetworkTableInstance.getDefault()
    var nt4Table: NetworkTable = nt4Instance.getTable("questnav")

    private val receiver = nt4Table.getIntegerTopic("miso").subscribe(0)
    private val transmitter = nt4Table.getIntegerTopic("mosi").publish()
    private val questResetPose = nt4Table.getDoubleArrayTopic("resetpose").publish()

    // Subscribe to the Network Tables questnav data topics
    private val questTimestamp: DoubleSubscriber = nt4Table.getDoubleTopic("timestamp").subscribe(0.0)
    private val questPosition: FloatArraySubscriber =
        nt4Table.getFloatArrayTopic("position").subscribe(floatArrayOf(0.0f, 0.0f, 0.0f))
    private val questQuaternion: FloatArraySubscriber =
        nt4Table.getFloatArrayTopic("quaternion").subscribe(floatArrayOf(0.0f, 0.0f, 0.0f, 0.0f))
    private val questEulerAngles: FloatArraySubscriber = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(
        floatArrayOf(0.0f, 0.0f, 0.0f)
    )
    private val questBatteryPercent: DoubleSubscriber = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0)

    private var timestampEpoch: Time? = null

    /**
     * The Quest's measured position.
     */
    val pose get() = questNavPose

    /**
     * The battery percent of the Quest.
     */
    val batteryPercent: Double
        get() = questBatteryPercent.get()

    /** If the Quest is connected. */
    val connected: Boolean
        get() = ((RobotController.getFPGATime() - questBatteryPercent.lastChange) / 1000) < 250

    /** The Quaternion of the Quest. */
    val quaternion: Quaternion
        get() {
            val qqFloats = questQuaternion.get()
            return Quaternion(
                qqFloats[0].toDouble(),
                qqFloats[1].toDouble(),
                qqFloats[2].toDouble(),
                qqFloats[3].toDouble()
            )
        }

    /** The Quests's timestamp. */
    val timestamp: Time
        get() {
            // Make sure our timestamp epochs are the same.
            val rawTimestamp = questTimestamp.get().seconds
            if (rawTimestamp != 0.seconds) {
                if (timestampEpoch == null) {
                    timestampEpoch = Timer.getTimestamp().seconds - rawTimestamp
                }
                return rawTimestamp + timestampEpoch
            }
            return 0.seconds
        }

    /**
     * Sends a command to the Quest.
     * @return `true` if the command was sent, `false` if the Quest was busy.
     */
    private fun sendCommand(command: QuestCommand): Boolean {
        if (receiver.get() == COMMAND_NEEDS_FINALIZATION) {
            return false
        }
        transmitter.set(command.id)
        return true
    }

    /**
     * Tell the Quest where it is on the field.
     */
    fun resetPosition(pose2d: Pose2d) {
        questResetPose.set(
            doubleArrayOf(
                pose2d.x,
                pose2d.y,
                pose2d.rotation.degrees,
            )
        )
        sendCommand(QuestCommand.PoseReset)
    }

    /**
     * Clean up the current commands so a new one could be sent.
     */
    fun finalizeCommands() {
        if (receiver.get() == COMMAND_NEEDS_FINALIZATION) {
            transmitter.set(QuestCommand.None.id)
        }
    }

    private val oculusYaw: Float
        // Get the yaw Euler angle of the headset
        get() = questEulerAngles.get()[1]

    private val questNavTranslation: Translation2d
        get() {
            val questNavPosition = questPosition.get()
            return Translation2d(questNavPosition[2].toDouble(), -questNavPosition[0].toDouble())
        }


    val poseOffset = Pose2d()

    private val questNavPose: Pose2d
        get() {
            val oculusPositionCompensated = questNavTranslation.minus(Translation2d(0.0, 0.1651)) // 6.5
            val transform = Transform2d(oculusPositionCompensated, Rotation2d.fromDegrees(oculusYaw.toDouble()))
            return poseOffset.transformBy(transform)
        }
}
