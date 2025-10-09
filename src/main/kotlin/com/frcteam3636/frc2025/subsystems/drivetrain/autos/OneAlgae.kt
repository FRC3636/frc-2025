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

class OneAlgae(val side: StartingPosition) : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = ALGAE_ONE

        return Commands.parallel(
            Commands.sequence(
                Drivetrain.driveToPointAllianceRelativeWithSlowZone(
                    reefPose,
                    DEFAULT_AUTO_CONSTRAINTS,
                    DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE,
                    SLOW_ZONE_DISTANCE,
                    SLOW_ZONE_ENTER_VELOCITY,
                    raisePoint = Elevator.Position.Stowed,
                ),
                Manipulator.intakeAlgae(),
                Commands.waitSeconds(1.0),
                Commands.parallel(
                    Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
                    Drivetrain.alignToBarge(),
                ),
                Robot.tossAlgae(),
            ),
        )
    }
}
