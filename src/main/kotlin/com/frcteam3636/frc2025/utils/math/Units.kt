package com.frcteam3636.frc2025.utils.math

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.Ultrasonic

// Number -> Measure

inline val Number.meters: Distance get() = Meters.of(this.toDouble())
inline val Number.cm: Distance get() = Centimeters.of(this.toDouble())
inline val Number.mm: Distance get() = Millimeters.of(this.toDouble())
inline val Number.inches: Distance get() = Inches.of(this.toDouble())
inline val Number.feet: Distance get() = Feet.of(this.toDouble())

inline val Number.rad: Angle get() = Radians.of(this.toDouble())
inline val Number.rotations: Angle get() = Rotations.of(this.toDouble())
inline val Number.deg: Angle get() = Degrees.of(this.toDouble())

inline val Number.rpm: AngularVelocity get() = RPM.of(this.toDouble())
inline val Number.rotationsPerSecond: AngularVelocity get() = RotationsPerSecond.of(this.toDouble())
inline val Number.radiansPerSecond: AngularVelocity get() = RadiansPerSecond.of(this.toDouble())
inline val Number.degreesPerSecond: AngularVelocity get() = DegreesPerSecond.of(this.toDouble())

inline val Number.metersPerSecond: LinearVelocity get() = MetersPerSecond.of(this.toDouble())
inline val Number.feetPerSecond: LinearVelocity get() = FeetPerSecond.of(this.toDouble())
inline val Number.inchesPerSecond: LinearVelocity get() = InchesPerSecond.of(this.toDouble())

inline val Number.volts: Voltage get() = Volts.of(this.toDouble())
inline val Number.millivolts: Voltage get() = Millivolts.of(this.toDouble())

inline val Number.voltsPerSecond: Velocity<VoltageUnit> get() = Volts.per(Second).of(this.toDouble())

inline val Number.amps: Current get() = Amps.of(this.toDouble())
inline val Number.milliamps: Current get() = Milliamps.of(this.toDouble())

inline val Number.seconds: Time get() = Seconds.of(this.toDouble())
inline val Number.milliseconds: Time get() = Milliseconds.of(this.toDouble())
inline val Number.minutes: Time get() = Minutes.of(this.toDouble())

inline val Number.pounds: Mass get() = Pounds.of(this.toDouble())
inline val Number.kilograms: Mass get() = Kilograms.of(this.toDouble())

inline val Number.kilogramSquareMeters: MomentOfInertia get() = KilogramSquareMeters.of(this.toDouble())

inline val Number.percent: Dimensionless get() = Percent.of(this.toDouble())

// Measure -> Number

inline fun Distance.inMeters() = `in`(Meters)
inline fun Distance.inCentimeters() = `in`(Centimeters)
inline fun Distance.inMillimeters() = `in`(Millimeter)
inline fun Distance.inInches() = `in`(Inches)
inline fun Distance.inFeet() = `in`(Feet)
fun Distance.toAngular(radius: Distance) = Radians.of(this.inMeters() / radius.inMeters())!!

inline fun Angle.inRadians() = `in`(Radians)
inline fun Angle.inRotations() = `in`(Rotations)
inline fun Angle.inDegrees() = `in`(Degrees)
fun Angle.toLinear(radius: Distance) = Meters.of(this.inRadians() * radius.inMeters())!!

inline fun AngularVelocity.inRPM() = `in`(RPM)
inline fun AngularVelocity.inRotationsPerSecond() = `in`(RotationsPerSecond)
inline fun AngularVelocity.inRadiansPerSecond() = `in`(RadiansPerSecond)
inline fun AngularVelocity.inDegreesPerSecond() = `in`(DegreesPerSecond)
fun AngularVelocity.toLinear(radius: Distance) = MetersPerSecond.of(this.inRadiansPerSecond() * radius.inMeters())!!

inline fun LinearVelocity.inMetersPerSecond() = `in`(MetersPerSecond)
inline fun LinearVelocity.inFeetPerSecond() = `in`(FeetPerSecond)
inline fun LinearVelocity.inInchesPerSecond() = `in`(InchesPerSecond)
fun LinearVelocity.toAngular(radius: Distance) = RadiansPerSecond.of(this.inMetersPerSecond() / radius.inMeters())!!

inline fun Voltage.inVolts() = `in`(Volts)
inline fun Voltage.inMillivolts() = `in`(Millivolts)

inline fun Current.inAmps() = `in`(Amps)
inline fun Current.inMilliamps() = `in`(Milliamps)

inline fun Time.inSeconds() = `in`(Seconds)
inline fun Time.inMilliseconds() = `in`(Milliseconds)
inline fun Time.inMinutes() = `in`(Minutes)

inline val Ultrasonic.range: Distance get() = Meters.of(rangeMM * 1000)
