package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.FIELD_LAYOUT
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.AprilTagTarget
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.utils.math.feet
import com.frcteam3636.frc2025.utils.math.inMeters
import com.frcteam3636.frc2025.utils.math.inMetersPerSecond
import com.frcteam3636.frc2025.utils.math.inches
import com.frcteam3636.frc2025.utils.math.metersPerSecond
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import kotlin.jvm.optionals.getOrDefault

open class AutoMode {
    open fun autoSequence(): Command {
        return autoSequence(true)
    }

    open fun autoSequence(shouldAutoStow: Boolean = true): Command {
        return Commands.none()
    }

    fun calculateAlliancePose(pose: Pose2d): Pose2d {
        return if (DriverStation.getAlliance()
                .getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
        ) pose else FlippingUtil.flipFieldPose(pose)
    }

    companion object Constants {
        val ALGAE_ONE = AprilTagTarget(21, Translation2d(1.5.inches.inMeters(), 0.0)).pose
        val ALGAE_TWO = AprilTagTarget(20, Translation2d(1.5.inches.inMeters(), 0.0)).pose
        val SLOW_ZONE_DISTANCE = 2.feet
        val SLOW_ZONE_ENTER_VELOCITY = 1.0.metersPerSecond
        val DEFAULT_AUTO_CONSTRAINTS = PathConstraints(10.0, 4.0, 2 * Math.PI, 4 * Math.PI)
        val AUTO_CONSTRAINTS_PICKUP = PathConstraints(4.75, 2.0, 2 * Math.PI, 4 * Math.PI)
        val DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE =
            PathConstraints(SLOW_ZONE_ENTER_VELOCITY.inMetersPerSecond(), 2.0, 2 * Math.PI, 4 * Math.PI)
        val LEFT_PIECE_ONE = AprilTagTarget(20, ReefBranchSide.Right).pose
        val LEFT_PIECE_TWO = AprilTagTarget(19, ReefBranchSide.Left).pose
        val LEFT_PIECE_THREE = AprilTagTarget(19, ReefBranchSide.Right).pose
        val LEFT_PICKUP_APRILTAG = AprilTagTarget(13, Translation2d.kZero).pose
        val LEFT_PICKUP = Pose2d(LEFT_PICKUP_APRILTAG.translation, LEFT_PICKUP_APRILTAG.rotation + Rotation2d.k180deg)
        val RIGHT_PIECE_ONE = AprilTagTarget(22, ReefBranchSide.Right).pose
        val RIGHT_PIECE_TWO = AprilTagTarget(17, ReefBranchSide.Left).pose
        val RIGHT_PIECE_THREE = AprilTagTarget(17, ReefBranchSide.Right).pose
        val RIGHT_PICKUP_APRILTAG = AprilTagTarget(12, Translation2d.kZero).pose
        val RIGHT_PICKUP =
            Pose2d(RIGHT_PICKUP_APRILTAG.translation, RIGHT_PICKUP_APRILTAG.rotation + Rotation2d.k180deg)
        val MIDDLE_PIECE_ONE = AprilTagTarget(21, ReefBranchSide.Left).pose
        val LEFT_STARTING_POSE = Pose2d(7.277, 6.183, Rotation2d.fromDegrees(180.0))
        val RIGHT_STARTING_POSE = Pose2d(7.277, 1.869, Rotation2d.fromDegrees(180.0))
        val REEF_BACKUP_DISTANCE = 8.inches
        val INTAKE_START_DISTANCE = 2.feet
        const val OUTTAKE_TIMEOUT = 0.3
    }
}

fun determineStartingPosition(): StartingPosition {
    val alliance = DriverStation.getAlliance()
        // 50/50 chance of being right lol.
        // unsure how of how else to handle this because if this function is called, and
        // we get a null value back we likely have bigger problems
        .getOrDefault(DriverStation.Alliance.Blue)
    val startingPosition = if (Drivetrain.estimatedPose.y > FIELD_LAYOUT.fieldWidth / 2) {
        if (alliance == DriverStation.Alliance.Blue)
            StartingPosition.Left
        else
            StartingPosition.Right
    } else {
        if (alliance == DriverStation.Alliance.Blue)
            StartingPosition.Right
        else
            StartingPosition.Left
    }
    return startingPosition
}

enum class StartingPosition {
    Left,
    Right
}