package com.frcteam3636.frc2025.subsystems.drivetrain.autos

import com.frcteam3636.frc2025.utils.math.degrees
import com.frcteam3636.frc2025.utils.math.degreesPerSecond
import com.frcteam3636.frc2025.utils.math.inRadians
import com.frcteam3636.frc2025.utils.math.inRadiansPerSecond
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

open class AutoMode(val name: String) {
    open fun autoSequence(): Command {
        return Commands.none()
    }

    open fun autoSequence(shouldAutoStow: Boolean = true): Command {
        return Commands.none()
    }

    companion object Constants {
        val DEFAULT_AUTO_CONSTRAINTS = PathConstraints(6.0, 4.0, 540.0.degreesPerSecond.inRadiansPerSecond(), 720.degrees.inRadians())
    }
}