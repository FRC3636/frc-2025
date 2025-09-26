package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.AprilTagTarget
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.funnel.Funnel
import com.frcteam3636.frc2025.subsystems.manipulator.CoralState
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.backup
import com.frcteam3636.frc2025.utils.math.degrees
import com.frcteam3636.frc2025.utils.math.feet
import com.frcteam3636.frc2025.utils.math.inches
import com.frcteam3636.frc2025.utils.math.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class TestAutoTwoCoralCurve() : AutoMode() {
    override fun autoSequence(): Command {
        val reefPose = AprilTagTarget(18, ReefBranchSide.Left).pose
        val firstReefPose = AprilTagTarget(18, ReefBranchSide.Right).pose
        val pickupPose = Pose2d(1.251.meters, 3.5.meters, Rotation2d.fromDegrees(45.0))

        return Commands.sequence(
            Commands.parallel(
                Drivetrain.driveToPointAllianceRelative(firstReefPose, DEFAULT_AUTO_CONSTRAINTS),
                Commands.sequence(
                    Commands.waitSeconds(ELEVATOR_DEPLOYMENT_TIME_FIRST_PIECE),
                    Elevator.setTargetHeight(Elevator.Position.HighBar)
                )
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Commands.parallel(
                Elevator.setTargetHeight(Elevator.Position.Stowed),
                Drivetrain.driveToPointAllianceRelative(pickupPose, DEFAULT_AUTO_CONSTRAINTS, firstReefPose.backup(2.feet))
            ),
            Commands.parallel(
                Commands.sequence(
                    Commands.race(
                        Commands.waitUntil {
                            Manipulator.coralState != CoralState.NONE
                        },
                        Commands.waitSeconds(5.0)
                    ),
                    Commands.parallel(
                        Drivetrain.driveToPointAllianceRelative(reefPose, DEFAULT_AUTO_CONSTRAINTS),
                        Commands.sequence(
                            Commands.waitSeconds(ELEVATOR_DEPLOYMENT_TIME_OTHER_PIECE),
                            Elevator.setTargetHeight(Elevator.Position.HighBar)
                        )
                    ),
                ),
                Commands.race(
                    Manipulator.intakeAuto(),
                    Funnel.intake()
                ).withTimeout(5.0),
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed),
        )
    }
}