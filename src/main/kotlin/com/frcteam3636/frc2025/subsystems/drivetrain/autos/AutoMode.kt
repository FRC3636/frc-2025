package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

open class AutoMode(val name: String) {
    open fun autoSequence(): Command {
        return Commands.none()
    }
}