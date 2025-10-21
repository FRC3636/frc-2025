package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class ThreeAlgae(val side: StartingPosition) : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {

        return Commands.sequence(
            TwoAlgae(side).autoSequence(),
            Commands.parallel(
                Elevator.setTargetHeight(Elevator.Position.Stowed),
                Drivetrain.driveToPointAllianceRelative(
                    target =
                ),
            ),
            Commands.race(
                Manipulator.intakeAlgaeAuto(),
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    Drivetrain.alignToBarge(false).alongWith(
                        Elevator.setTargetHeight(Elevator.Position.Stowed),
                    ),
                ),
            ),
            Robot.tossAlgae(),
        )
    }
}