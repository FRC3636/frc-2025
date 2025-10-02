package com.frcteam3636.frc2025

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Dashboard {
    val autoChooser = SendableChooser<AutoModes>().apply {
        for (autoMode in AutoModes.entries) {
            if (autoMode == AutoModes.None)
                setDefaultOption(autoMode.autoName, autoMode)
            else if (Preferences.getBoolean(
                    "developerMode",
                    true
                ) && autoMode.developerAuto && !DriverStation.isFMSAttached()
            ) {
                addOption(autoMode.autoName, autoMode)
            } else if (!autoMode.developerAuto)
                addOption(autoMode.autoName, autoMode)
        }
    }

    fun initialize() {
        SmartDashboard.putData(autoChooser)
    }
}

enum class AutoModes(val autoName: String, val developerAuto: Boolean = false, val sideRequired: Boolean = true) {
    None("None", sideRequired = false),
    OnePieceCoral("One Piece Coral"),
    TwoPieceCoral("Two Piece Coral"),
    ThreePieceCoral("Three Piece Coral"),
    FourPieceCoral("Four Piece Coral"),
    OnePieceCoralMiddle("One Piece Coral Middle", sideRequired = false),
    TestAutoOneCoral("Test Auto 1 Coral", true, false),
    TestAutoTwoCoral("Test Auto 2 Coral", true, false),
    TestAutoThreeCoral("Test Auto 3 Coral", true, false),
}