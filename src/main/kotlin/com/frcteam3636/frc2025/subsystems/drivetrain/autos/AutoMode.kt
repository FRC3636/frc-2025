package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.poi.AprilTagTarget
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.utils.math.degrees
import com.frcteam3636.frc2025.utils.math.degreesPerSecond
import com.frcteam3636.frc2025.utils.math.inRadians
import com.frcteam3636.frc2025.utils.math.inRadiansPerSecond
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

open class AutoMode(val name: String) {
    open fun autoSequence(): Command {
        return Commands.none()
    }

    open fun autoSequence(shouldAutoStow: Boolean = true): Command {
        return Commands.none()
    }

    companion object Constants {
        val DEFAULT_AUTO_CONSTRAINTS = PathConstraints(6.0, 4.0, 540.0.degreesPerSecond.inRadiansPerSecond(), 720.degrees.inRadians())
        val LEFT_PIECE_ONE = AprilTagTarget(20, ReefBranchSide.Right).pose
        val LEFT_PIECE_TWO = AprilTagTarget(19, ReefBranchSide.Left).pose
        val LEFT_PIECE_THREE = AprilTagTarget(19, ReefBranchSide.Right).pose
        val LEFT_PICKUP = AprilTagTarget(13, Translation2d.kZero).pose
        val RIGHT_PIECE_ONE = AprilTagTarget(22, ReefBranchSide.Right).pose
        val RIGHT_PIECE_TWO = AprilTagTarget(17, ReefBranchSide.Left).pose
        val RIGHT_PIECE_THREE = AprilTagTarget(17, ReefBranchSide.Right).pose
        val RIGHT_PICKUP = AprilTagTarget(12, Translation2d.kZero).pose
        const val OUTTAKE_TIMEOUT = 0.3
        const val INTAKE_TIMEOUT = 3.0
        const val CORAL_INTAKE_LEAVE_TIMEOUT = 1.0
        const val ELEVATOR_DEPLOYMENT_TIME = 1.5
    }
}

enum class StartingPosition {
    Left,
    Middle,
    Right
}