package com.frcteam3636.frc2025

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkFlex

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

    RollerMotor(10),
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
    ElevatorMotor(11, "*"),
    ManipulatorMotor(12, "*"),
    PigeonGyro(20, "*"),
    ElevatorEncoder(30, "*"),
    ClimbMotor(13, "*"),
    ClimbEncoder(14, "*"),
}

fun CANcoder(id: CTREDeviceId) = CANcoder(id.num, id.bus)
fun TalonFX(id: CTREDeviceId) = TalonFX(id.num, id.bus)
fun Pigeon2(id: CTREDeviceId) = Pigeon2(id.num, id.bus)