@file:Suppress("unused")
package com.frcteam3636.frc2025

import au.grapplerobotics.LaserCan
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkFlex
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax

// This module contains one enum for each (device type, manufacturer) pair we use.

enum class REVMotorControllerId(val num: Int) {
    FrontLeftTurningMotor(5),
    BackLeftTurningMotor(6),
    BackRightTurningMotor(7),
    FrontRightTurningMotor(8),

    FrontLeftDrivingMotor(1),
    BackLeftDrivingMotor(2),
    BackRightDrivingMotor(3),
    FrontRightDrivingMotor(4),
}

enum class GrappleRoboticsDeviceId(val num: Int) {
    ManipulatorLaserCAN(21),
}

fun SparkMax(id: REVMotorControllerId, type: SparkLowLevel.MotorType) =
    SparkMax(id.num, type)

fun SparkFlex(id: REVMotorControllerId, type: SparkLowLevel.MotorType) =
    SparkFlex(id.num, type)

enum class CTREDeviceId(val num: Int, val bus: String) {
    FrontLeftDrivingMotor(1, "*"),
    BackLeftDrivingMotor(2, "*"),
    BackRightDrivingMotor(3, "*"),
    FrontRightDrivingMotor(4, "*"),
    LeftElevatorMotor(11, "*"),
    RightElevatorMotor(12, "*"),
    ManipulatorMotor(13, "*"),
    FunnelMotor(14, "*"),
    PigeonGyro(20, "*"),
    ElevatorEncoder(30, "*"),
    CANRange(62, "rio")
}

fun CANcoder(id: CTREDeviceId) = CANcoder(id.num, id.bus)
fun TalonFX(id: CTREDeviceId) = TalonFX(id.num, id.bus)
fun Pigeon2(id: CTREDeviceId) = Pigeon2(id.num, id.bus)
fun Lasercan(id: GrappleRoboticsDeviceId) = LaserCan(id.num)
fun CANrange(id: CTREDeviceId) = CANrange(id.num, id.bus)