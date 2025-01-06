package com.frcteam3636.frc2024.utils

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Joystick

// Joystick x/y are inverted from the standard coordinate system
val Joystick.translation2d get() = Translation2d(-y, -x)
