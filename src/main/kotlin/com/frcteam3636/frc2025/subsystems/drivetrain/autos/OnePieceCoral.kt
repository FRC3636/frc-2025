package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class OnePieceCoral(val side: StartingPosition) : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = if (side == StartingPosition.Left) LEFT_PIECE_ONE else RIGHT_PIECE_ONE

        return Commands.sequence(
            Commands.parallel(
                Drivetrain.driveToPointAllianceRelative(reefPose, DEFAULT_AUTO_CONSTRAINTS),
                Commands.sequence(
                    Commands.waitSeconds(ELEVATOR_DEPLOYMENT_TIME_FIRST_PIECE),
                    Elevator.setTargetHeight(Elevator.Position.HighBar)
                )
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}