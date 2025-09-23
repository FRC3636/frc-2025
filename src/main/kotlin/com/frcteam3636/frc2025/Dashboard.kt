package com.frcteam3636.frc2025

import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.littletonrobotics.junction.Logger

object Dashboard {
    val autoChooser = SendableChooser<AutoModes>().apply {
        for (autoMode in AutoModes.entries) {
            if (autoMode == AutoModes.None)
                setDefaultOption(autoMode.autoName, autoMode)
            else
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

enum class AutoModes(val autoName: String) {
    None("None"),
    OnePieceCoral("One Piece Coral"),
    TwoPieceCoral("Two Piece Coral"),
    ThreePieceCoral("Three Piece Coral"),
    TestAuto("Test Auto"),
    TestAutoPickup("Test Auto w/ HP Pickup")
}