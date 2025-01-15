package com.frcteam3636.frc2025.utils

import com.frcteam3636.frc2025.Robot
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
        return when (DriverStation.getAlliance().getOrNull()) {
            Alliance.Red -> -base
            else -> base
        }
    }

/**
 * Returns the translation of the joystick input.
 */
val Joystick.translation2d: Translation2d
    // The field-space translation returned by this method is rotated 90 degrees from the joystick's
    // perspective. (x, y) -> (y, -x) The joystick's Y-axis is also inverted because of our physical
    // hardware, but this isn't an issue in simulation.
    get() = Translation2d(
        if (Robot.model == Robot.Model.SIMULATION) {
            y
        } else {
            -y
        },
        -x
    )
