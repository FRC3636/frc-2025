package com.frcteam3636.frc2025

import com.frcteam3636.frc2025.subsystems.drivetrain.autos.OnePieceCoral
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.StartingPosition
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.ThreePieceCoral
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.TwoPieceCoral
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.Logger

object Dashboard {
    val autoChooser = SendableChooser<AutoModes>().apply {
        for (autoMode in AutoModes.entries) {
            addOption(autoMode.name, autoMode)
        }
    }
    val defaultAuto: Command? = Commands.none()


    fun initialize() {
        PathPlannerLogging.setLogTargetPoseCallback {
            Logger.recordOutput("/Drivetrain/Target Pose", it)
        }
        PathPlannerLogging.setLogActivePathCallback {
            Logger.recordOutput("/Drivetrain/Desired Path", *it.toTypedArray())
        }
        SmartDashboard.putData(autoChooser)
    }
}

enum class AutoModes(name: String) {
    None("None"),
    OnePieceCoral("One Piece Coral"),
    TwoPieceCoral("Two Piece Coral"),
    ThreePieceCoral("Three Piece Coral"),
}