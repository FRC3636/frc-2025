package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator import com.frcteam3636.frc2025.utils.math.calculateAlliancePose
import com.frcteam3636.frc2025.utils.math.feet
import com.frcteam3636.frc2025.utils.math.metersPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class OnePieceCoral(val side: StartingPosition) : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = if (side == StartingPosition.Left) LEFT_PIECE_ONE else RIGHT_PIECE_ONE
        val thresholdPose = calculateAlliancePose(reefPose)

        return Commands.sequence(
            Commands.parallel(
                Drivetrain.driveToPointAllianceRelative(
                    reefPose,
                    DEFAULT_AUTO_CONSTRAINTS,
//                DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE,
//                SLOW_ZONE_DISTANCE,
//                0.0.metersPerSecond
                ),
                Commands.sequence(
                    Commands.waitUntil {
                        Drivetrain.estimatedPose.translation.getDistance(thresholdPose.translation).feet < ELEVATOR_DEPLOY_DISTANCE
                    },
                    Elevator.setTargetHeight(Elevator.Position.HighBar)
                )
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}