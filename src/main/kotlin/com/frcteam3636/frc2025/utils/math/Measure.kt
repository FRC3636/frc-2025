package com.frcteam3636.frc2025.utils.math

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkClosedLoopController
import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*

var RelativeEncoder.angle: Angle
    get() = Rotations.of(position)
    set(value) {
        position = value.`in`(Rotations)
    }

val RelativeEncoder.angularVelocity: AngularVelocity
    get() = RPM.of(velocity)

val AbsoluteEncoder.angle: Angle
    get() = Rotations.of(position)

val AbsoluteEncoder.angularVelocity: AngularVelocity
    get() = RPM.of(velocity)

fun SparkClosedLoopController.setReference(
    value: Angle,
    unit: AngleUnit,
) = setReference(value.`in`(unit), SparkBase.ControlType.kPosition)

fun SparkClosedLoopController.setReference(
    value: Distance,
    unit: DistanceUnit,
) = setReference(value.`in`(unit), SparkBase.ControlType.kPosition)

fun SparkClosedLoopController.setReference(
    value: LinearVelocity,
    unit: LinearVelocityUnit,
) = setReference(value.`in`(unit), SparkBase.ControlType.kVelocity)

fun SparkClosedLoopController.setReference(
    value: AngularVelocity,
    unit: AngularVelocityUnit,
) = setReference(value.`in`(unit), SparkBase.ControlType.kVelocity)