package com.frcteam3636.frc2025.utils.math

import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

const val TAU = PI * 2

fun Translation2d.fromPolar(magnitude: Double, angle: Double): Translation2d {
    return Translation2d(magnitude * cos(angle), magnitude * sin(angle))
}

fun Translation2d.dot(other: Translation2d): Double {
    return x * other.x + y * other.y
}
