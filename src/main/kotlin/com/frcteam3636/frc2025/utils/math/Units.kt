package com.frcteam3636.frc2025.utils.math

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage

val Distance.meters get() = `in`(Meters)
val Distance.centimeters get() = `in`(Centimeters)
val Distance.inches get() = `in`(Inches)
val Distance.feet get() = `in`(Feet)
fun Distance.toAngular(radius: Distance) = Radians.of(this.meters / radius.meters)!!

val Angle.radians get() = `in`(Radians)
val Angle.rotations get() = `in`(Rotations)
val Angle.degrees get() = `in`(Degrees)
fun Angle.toLinear(radius: Distance) = Meters.of(this.radians * radius.meters)!!

val AngularVelocity.rpm get() = `in`(RPM)
val AngularVelocity.rotationsPerSecond get() = `in`(RotationsPerSecond)
val AngularVelocity.radiansPerSecond get() = `in`(RadiansPerSecond)
val AngularVelocity.degreesPerSecond get() = `in`(DegreesPerSecond)
fun AngularVelocity.toLinear(radius: Distance) = MetersPerSecond.of(this.radiansPerSecond * radius.meters)!!

val LinearVelocity.metersPerSecond get() = `in`(MetersPerSecond)
val LinearVelocity.feetPerSecond get() = `in`(FeetPerSecond)
val LinearVelocity.inchesPerSecond get() = `in`(InchesPerSecond)
fun LinearVelocity.toAngular(radius: Distance) = RadiansPerSecond.of(this.metersPerSecond / radius.meters)!!

val Voltage.volts get() = `in`(Volts)
val Voltage.millivolts get() = `in`(Millivolts)

val Current.amps get() = `in`(Amps)
val Current.milliamps get() = `in`(Milliamps)

val Time.seconds get() = `in`(Seconds)
val Time.milliseconds get() = `in`(Milliseconds)
val Time.minutes get() = `in`(Minutes)