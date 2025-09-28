package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.funnel.Funnel
import com.frcteam3636.frc2025.subsystems.manipulator.CoralState
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.feet
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class FourPieceCoral(val side: StartingPosition) : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = if (side == StartingPosition.Left) LEFT_PIECE_THREE else RIGHT_PIECE_THREE
        val pickupPose = if (side == StartingPosition.Left) LEFT_PICKUP else RIGHT_PICKUP

        return Commands.sequence(
            ThreePieceCoral(side).autoSequence(false),
            Commands.parallel(
                Commands.sequence(
                    Drivetrain.driveToPointAllianceRelative(pickupPose, DEFAULT_AUTO_CONSTRAINTS),
                    Commands.waitUntil {
                        Manipulator.coralState != CoralState.NONE
                    },
                    Drivetrain.driveToPointAllianceRelativeWithSlowConstraintZone(reefPose, DEFAULT_AUTO_CONSTRAINTS, DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE,SLOW_ZONE_DISTANCE,
                        raisePoint = Elevator.Position.MidBar),
                ),
                Elevator.setTargetHeight(Elevator.Position.Stowed),
                Commands.sequence(
                    Commands.waitUntil {
                        Drivetrain.estimatedPose.translation.getDistance(pickupPose.translation).feet < 2.feet
                    },
                    Commands.race(
                        Manipulator.intakeAuto(),
                        Funnel.intake(),
                    )
                )
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}