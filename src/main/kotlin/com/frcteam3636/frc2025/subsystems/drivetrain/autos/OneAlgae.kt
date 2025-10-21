package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.funnel.Funnel
import com.frcteam3636.frc2025.subsystems.manipulator.CoralState
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.feet
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class OneAlgae() : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {

        return Commands.sequence(
            OnePieceCoralMiddle().autoSequence(false),
            Commands.parallel(
                Elevator.setTargetHeight(Elevator.Position.Stowed),
                Drivetrain.alignToReefAlgae(false),
            ),
            Commands.race(
                Manipulator.intakeAlgaeAuto(),
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    Drivetrain.alignToBarge(false),
                ),
            ),
            Robot.tossAlgae(),
        )
    }
}
