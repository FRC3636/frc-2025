package com.frcteam3636.frc2025.utils

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Joystick
import kotlin.jvm.optionals.getOrNull


/**
 * Returns the translation of the joystick input, flipped to match the current alliance.
 */
val Joystick.fieldRelativeTranslation2d: Translation2d
    get() {
        val base = translation2d
        // Joystick x/y are inverted from the standard coordinate system+
        return when (DriverStation.getAlliance().getOrNull()) {
            Alliance.Red -> -base
            else -> base
        }
    }

/**
 * Returns the translation of the joystick input.
 */
val Joystick.translation2d: Translation2d // Joystick x/y are inverted from the standard coordinate system
    get() = Translation2d(-y, -x)
