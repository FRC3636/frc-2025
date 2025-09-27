package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.funnel.Funnel
import com.frcteam3636.frc2025.subsystems.manipulator.CoralState
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.backup
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class TwoPieceCoral(val side: StartingPosition) : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = if (side == StartingPosition.Left) LEFT_PIECE_TWO else RIGHT_PIECE_TWO
        val firstReefPose = if (side == StartingPosition.Left) LEFT_PIECE_ONE else RIGHT_PIECE_ONE
        val pickupPose = if (side == StartingPosition.Left) LEFT_PICKUP else RIGHT_PICKUP

        return Commands.sequence(
            OnePieceCoral(side).autoSequence(false),
            Commands.parallel(
                Elevator.setTargetHeight(Elevator.Position.Stowed),
                Drivetrain.driveToPointAllianceRelativeWithMiddlePoint(pickupPose, DEFAULT_AUTO_CONSTRAINTS, firstReefPose.backup(REEF_BACKUP_DISTANCE))
            ),
            Commands.parallel(
                Commands.sequence(
                    Commands.race(
                        Commands.waitUntil {
                            Manipulator.coralState != CoralState.NONE
                        },
                        Commands.waitSeconds(CORAL_INTAKE_LEAVE_TIMEOUT)
                    ),
                    Drivetrain.driveToPointAllianceRelativeWithSlowZone(reefPose, DEFAULT_AUTO_CONSTRAINTS, DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE,SLOW_ZONE_DISTANCE, SLOW_ZONE_ENTER_VELOCITY),
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