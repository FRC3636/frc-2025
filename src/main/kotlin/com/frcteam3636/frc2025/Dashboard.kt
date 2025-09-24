package com.frcteam3636.frc2025

import com.frcteam3636.frc2025.subsystems.drivetrain.autos.TestAutoTwoCoral
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.littletonrobotics.junction.Logger

object Dashboard {
    val autoChooser = SendableChooser<AutoModes>().apply {
        for (autoMode in AutoModes.entries) {
            if (autoMode == AutoModes.None)
                setDefaultOption(autoMode.autoName, autoMode)
            else if (Preferences.getBoolean("developerMode", true) && autoMode.developerAuto && !DriverStation.isFMSAttached()) {
                addOption(autoMode.autoName, autoMode)
            } else if (!autoMode.developerAuto)
                addOption(autoMode.autoName, autoMode)
        }
    }

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

enum class AutoModes(val autoName: String, val developerAuto: Boolean = false) {
    None("None"),
    OnePieceCoral("One Piece Coral"),
    TwoPieceCoral("Two Piece Coral"),
    ThreePieceCoral("Three Piece Coral"),
    TestAutoOneCoral("Test Auto 1 Coral", true),
    TestAutoTwoCoral("Test Auto 2 Coral", true)
}