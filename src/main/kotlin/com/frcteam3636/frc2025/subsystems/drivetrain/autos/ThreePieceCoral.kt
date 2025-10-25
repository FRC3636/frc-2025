package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.funnel.Funnel
import com.frcteam3636.frc2025.subsystems.manipulator.CoralState
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.calculateAlliancePose
import com.frcteam3636.frc2025.utils.math.feet
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class ThreePieceCoral(val side: StartingPosition) : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = if (side == StartingPosition.Left) LEFT_PIECE_THREE else RIGHT_PIECE_THREE
        val pickupPose = if (side == StartingPosition.Left) LEFT_PICKUP else RIGHT_PICKUP

        val thresholdPose = calculateAlliancePose(pickupPose)
        val elevatorThresholdPose = calculateAlliancePose(reefPose)

        return Commands.sequence(
            TwoPieceCoral(side).autoSequence(false),
            Commands.parallel(
                Commands.sequence(
                    Drivetrain.driveToPointAllianceRelative(pickupPose, AUTO_CONSTRAINTS_PICKUP).alongWith(
                        Elevator.setTargetHeight(Elevator.Position.Stowed)
                    ),
                    Commands.waitUntil {
                        Manipulator.coralState != CoralState.NONE
                    }.withTimeout(INTAKE_TIMEOUT),
                    Commands.parallel(
                        Drivetrain.driveToPointAllianceRelative(
                            reefPose,
                            DEFAULT_AUTO_CONSTRAINTS,
                        ),
                        Commands.sequence(
                            Commands.waitUntil {
                                Drivetrain.estimatedPose.translation.getDistance(elevatorThresholdPose.translation).feet < ELEVATOR_DEPLOY_DISTANCE && Manipulator.coralState == CoralState.HELD
                            },
                            Elevator.setTargetHeight(Elevator.Position.HighBar)
                        )
                    ),
                ),
                Commands.sequence(
                    Commands.waitUntil {
                        Drivetrain.estimatedPose.translation.getDistance(thresholdPose.translation).feet < INTAKE_START_DISTANCE
                    },
                    Commands.race(
                        Manipulator.intakeAuto(),
                        Funnel.intake(),
                    ).withTimeout(INTAKE_TIMEOUT),
                    Commands.either(
                        Commands.none(),
                        Commands.sequence(
                            Commands.waitSeconds(INTAKE_RESTART_TIME),
                            Commands.race(
                                Manipulator.intakeAuto(),
                                Funnel.intake(),
                            ),
                        ),
                        { Manipulator.coralState == CoralState.HELD }
                    )
                )
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}