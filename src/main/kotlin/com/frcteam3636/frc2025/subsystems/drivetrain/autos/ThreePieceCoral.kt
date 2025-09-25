package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.funnel.Funnel
import com.frcteam3636.frc2025.subsystems.manipulator.CoralState
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.backup
import com.frcteam3636.frc2025.utils.math.inches
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class ThreePieceCoral(val side: StartingPosition) : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = if (side == StartingPosition.Left) LEFT_PIECE_THREE else RIGHT_PIECE_THREE
        val pickupPose = if (side == StartingPosition.Left) LEFT_PICKUP else RIGHT_PICKUP

        return Commands.sequence(
            TwoPieceCoral(side).autoSequence(false),
            Commands.parallel(
                Elevator.setTargetHeight(Elevator.Position.Stowed),
                Drivetrain.driveToPointAllianceRelative(pickupPose, DEFAULT_AUTO_CONSTRAINTS)
            ),
            Commands.parallel(
                Commands.sequence(
                    Commands.race(
                        Commands.waitUntil {
                            Manipulator.coralState != CoralState.NONE
                        },
                        Commands.waitSeconds(CORAL_INTAKE_LEAVE_TIMEOUT)
                    ),
                    Commands.parallel(
                        Drivetrain.driveToPointAllianceRelative(reefPose, DEFAULT_AUTO_CONSTRAINTS),
                        Commands.sequence(
                            Commands.waitSeconds(ELEVATOR_DEPLOYMENT_TIME),
                            Elevator.setTargetHeight(Elevator.Position.HighBar)
                        )
                    ),
                ),
                Commands.race(
                    Manipulator.intakeAuto(),
                    Funnel.intake()
                ).withTimeout(INTAKE_TIMEOUT),
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}