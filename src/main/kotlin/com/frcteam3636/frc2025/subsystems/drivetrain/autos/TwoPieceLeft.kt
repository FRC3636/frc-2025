package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.AprilTagTarget
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.funnel.Funnel
import com.frcteam3636.frc2025.subsystems.manipulator.CoralState
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

object TwoPieceLeft : AutoMode("2 Piece Left") {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        return Commands.sequence(
            OnePieceLeft.autoSequence(),
            Commands.parallel(
                Elevator.setTargetHeight(Elevator.Position.Stowed),
                Drivetrain.driveToPointAllianceRelative(AprilTagTarget(13, Translation2d.kZero).pose, DEFAULT_AUTO_CONSTRAINTS)
            ),
            Commands.parallel(
                Commands.sequence(
                    Commands.race(
                        Commands.waitUntil {
                            Manipulator.coralState != CoralState.NONE
                        },
                        Commands.waitSeconds(1.0)
                    ),
                    Commands.parallel(
                        Drivetrain.driveToPointAllianceRelative(AprilTagTarget(19, ReefBranchSide.Left).pose, DEFAULT_AUTO_CONSTRAINTS),
                        Commands.sequence(
                            Commands.waitSeconds(3.0),
                            Elevator.setTargetHeight(Elevator.Position.HighBar)
                        )
                    ),
                ),
                Commands.race(
                    Manipulator.intakeAuto(),
                    Funnel.intake()
                ).withTimeout(3.0),
            ),
            Manipulator.outtake().withTimeout(0.3),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}