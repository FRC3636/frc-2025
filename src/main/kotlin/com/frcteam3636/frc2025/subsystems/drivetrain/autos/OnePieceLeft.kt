package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.AprilTagTarget
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

object OnePieceLeft : AutoMode("1 Piece Left") {
    override fun autoSequence(): Command {
        return Commands.sequence(
            Drivetrain.driveToPointAllianceRelative(AprilTagTarget(20, ReefBranchSide.Left).pose),
            Elevator.setTargetHeight(Elevator.Position.HighBar),
            Manipulator.outtake(),
            Elevator.setTargetHeight(Elevator.Position.Stowed),
        )
    }
}