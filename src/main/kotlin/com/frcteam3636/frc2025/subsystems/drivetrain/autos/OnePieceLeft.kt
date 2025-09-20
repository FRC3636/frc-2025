package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.AprilTagTarget
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.degrees
import com.frcteam3636.frc2025.utils.math.degreesPerSecond
import com.frcteam3636.frc2025.utils.math.degreesPerSecondPerSecond
import com.frcteam3636.frc2025.utils.math.inDegrees
import com.frcteam3636.frc2025.utils.math.inRadians
import com.frcteam3636.frc2025.utils.math.inRadiansPerSecond
import com.frcteam3636.frc2025.utils.math.radiansPerSecond
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

object OnePieceLeft : AutoMode("1 Piece Left") {
    override fun autoSequence(): Command {
        return Commands.sequence(
            Commands.parallel(
                Drivetrain.driveToPointAllianceRelative(AprilTagTarget(20, ReefBranchSide.Left).pose, PathConstraints(6.0, 4.0, 540.0.degreesPerSecond.inRadiansPerSecond(), 720.degrees.inRadians())),
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    Elevator.setTargetHeight(Elevator.Position.HighBar)
                )
            ),
            Manipulator.outtake().withTimeout(0.3),
            Elevator.setTargetHeight(Elevator.Position.Stowed),
        )
    }
}