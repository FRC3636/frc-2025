package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class ThreeAlgae() : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {

        return Commands.sequence(
            TwoAlgae().autoSequence(),
            Commands.parallel(
                Elevator.setTargetHeight(Elevator.Position.Stowed),
                Drivetrain.driveToPointAllianceRelative(
                    Algae_Three_Approach_Align,
                ),
                Drivetrain.alignToReefAlgae(
                    false,
                )
            ),
            Commands.race(
                Manipulator.intakeAlgaeAuto(),
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    Drivetrain.driveToPointAllianceRelative(Algae_Three_Leaving_Align).alongWith(
                        Elevator.setTargetHeight(Elevator.Position.Stowed),
                    ),
                    Drivetrain.alignToBarge(false)
                ),
            ),
            Robot.tossAlgae(),
        )
    }
}