package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.backup
import com.frcteam3636.frc2025.utils.math.feet
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class OneCoralMiddleAlgaePickup : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        return Commands.sequence(
            OnePieceCoralMiddle().autoSequence(false),
            Commands.parallel(
                Elevator.setTargetHeight(Elevator.Position.Stowed),
                Drivetrain.driveToPointAllianceRelative(
                    MIDDLE_PIECE_ONE.backup(0.5.feet),
                    DEFAULT_AUTO_CONSTRAINTS
                ),
            ),
            Drivetrain.driveToPointAllianceRelativeWithSlowConstraintZone(
                MIDDLE_ALGAE_PICKUP,
                DEFAULT_AUTO_CONSTRAINTS,
                DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE,
                SLOW_ZONE_DISTANCE,
                shouldRaise = false
            ),
            Manipulator.intakeAlgae()
        )
    }
}