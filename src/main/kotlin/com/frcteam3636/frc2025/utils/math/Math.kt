@file:Suppress("unused", "UnusedReceiverParameter")
package com.frcteam3636.frc2025.utils.math

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
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

fun Pose2d.backup(distance: Distance): Pose2d {
    return this + Transform2d(Translation2d(-distance, 0.meters), Rotation2d.kZero)
}