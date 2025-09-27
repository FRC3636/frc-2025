package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.AprilTagTarget
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.feet
import com.frcteam3636.frc2025.utils.math.metersPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class TestAuto() : AutoMode() {
    override fun autoSequence(shouldAutoStow: Boolean): Command {
        val reefPose = AprilTagTarget(18, ReefBranchSide.Right).pose

        return Commands.sequence(
            Drivetrain.driveToPointAllianceRelativeWithSlowZone(reefPose, DEFAULT_AUTO_CONSTRAINTS, DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE,SLOW_ZONE_DISTANCE, SLOW_ZONE_ENTER_VELOCITY),
            Manipulator.outtake().withTimeout(OUTTAKE_TIMEOUT),
            Elevator.setTargetHeight(Elevator.Position.Stowed).onlyIf { shouldAutoStow },
        )
    }
}