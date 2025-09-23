package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.AprilTagTarget
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class TestAuto() : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = AprilTagTarget(18, ReefBranchSide.Right).pose

        return Commands.sequence(
            Commands.parallel(
                Drivetrain.driveToPointAllianceRelative(reefPose, DEFAULT_AUTO_CONSTRAINTS),
                Commands.sequence(
                    Commands.waitSeconds(ELEVATOR_DEPLOYMENT_TIME),
                    Elevator.setTargetHeight(Elevator.Position.HighBar)
                )
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}