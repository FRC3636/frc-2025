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

inline val Distance.inMeters get() = `in`(Meters)
inline val Distance.inCentimeters get() = `in`(Centimeters)
inline val Distance.inMillimeters get() = `in`(Millimeter)
inline val Distance.inInches get() = `in`(Inches)
inline val Distance.inFeet get() = `in`(Feet)
fun Distance.toAngular(radius: Distance) = Radians.of(this.inMeters / radius.inMeters)!!

inline val Angle.inRadians get() = `in`(Radians)
inline val Angle.inRotations get() = `in`(Rotations)
inline val Angle.inDegrees get() = `in`(Degrees)
fun Angle.toLinear(radius: Distance) = Meters.of(this.inRadians * radius.inMeters)!!

inline val AngularVelocity.inRPM get() = `in`(RPM)
inline val AngularVelocity.inRotationsPerSecond get() = `in`(RotationsPerSecond)
inline val AngularVelocity.inRadiansPerSecond get() = `in`(RadiansPerSecond)
inline val AngularVelocity.inDegreesPerSecond get() = `in`(DegreesPerSecond)
fun AngularVelocity.toLinear(radius: Distance) = MetersPerSecond.of(this.inRadiansPerSecond * radius.inMeters)!!

inline val LinearVelocity.inMetersPerSecond get() = `in`(MetersPerSecond)
inline val LinearVelocity.inFeetPerSecond get() = `in`(FeetPerSecond)
inline val LinearVelocity.inInchesPerSecond get() = `in`(InchesPerSecond)
fun LinearVelocity.toAngular(radius: Distance) = RadiansPerSecond.of(this.inMetersPerSecond / radius.inMeters)!!

inline val Voltage.inVolts get() = `in`(Volts)
inline val Voltage.inMillivolts get() = `in`(Millivolts)

inline val Current.inAmps get() = `in`(Amps)
inline val Current.inMilliamps get() = `in`(Milliamps)

inline val Time.inSeconds get() = `in`(Seconds)
inline val Time.inMilliseconds get() = `in`(Milliseconds)
inline val Time.inMinutes get() = `in`(Minutes)

inline val Ultrasonic.range: Distance get() = Meters.of(rangeMM * 1000)
