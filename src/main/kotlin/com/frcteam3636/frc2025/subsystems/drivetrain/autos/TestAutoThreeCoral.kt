package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.AprilTagTarget
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.funnel.Funnel
import com.frcteam3636.frc2025.subsystems.manipulator.CoralState
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class TestAutoThreeCoral() : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = AprilTagTarget(18, ReefBranchSide.Left).pose
        val pickupPose = Pose2d(1.251.meters, 4.034.meters, Rotation2d.kZero)

        return Commands.sequence(
            TestAutoTwoCoral().autoSequence(false),
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
                        Commands.waitSeconds(5.0)
                    ),
                    Drivetrain.driveToPointAllianceRelativeWithSlowZone(
                        reefPose,
                        DEFAULT_AUTO_CONSTRAINTS,
                        DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE,
                        SLOW_ZONE_DISTANCE,
                        SLOW_ZONE_ENTER_VELOCITY,
                        raisePoint = Elevator.Position.MidBar
                    ),
                ),
                Commands.race(
                    Manipulator.intakeAuto(),
                    Funnel.intake()
                ).withTimeout(5.0),
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}