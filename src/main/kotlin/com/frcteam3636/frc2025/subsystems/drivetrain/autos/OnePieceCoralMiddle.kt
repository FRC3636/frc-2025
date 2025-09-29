package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class OnePieceCoralMiddle : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        return Commands.sequence(
            Drivetrain.driveToPointAllianceRelativeWithSlowZone(
                MIDDLE_PIECE_ONE,
                DEFAULT_AUTO_CONSTRAINTS,
                DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE,
                SLOW_ZONE_DISTANCE,
                SLOW_ZONE_ENTER_VELOCITY
            ),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}