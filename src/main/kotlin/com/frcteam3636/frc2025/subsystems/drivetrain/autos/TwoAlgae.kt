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

class TwoAlgae(val side: StartingPosition) : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = ALGAE_ONE
        var shouldStopIntake = false

        return Commands.sequence(
            OneAlgae(side).autoSequence(),
            Commands.parallel(
                Manipulator.intakeAlgae().onlyWhile {!shouldStopIntake},
                Commands.sequence(
                    Drivetrain.driveToPointAllianceRelativeWithSlowZone(
                        reefPose,
                        DEFAULT_AUTO_CONSTRAINTS,
                        DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE,
                        SLOW_ZONE_DISTANCE,
                        SLOW_ZONE_ENTER_VELOCITY,
                        raisePoint = Elevator.Position.Stowed,
                    ),

                    Commands.parallel(
                        Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
                        Drivetrain.alignToBarge(),
                    ),
                    Commands.runOnce(
                        {shouldStopIntake = true}
                    ),
                    Robot.tossAlgae(),
                ),
            ),
        )
    }
}