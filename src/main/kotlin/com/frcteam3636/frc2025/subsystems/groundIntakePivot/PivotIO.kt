package com.frcteam3636.frc2025.subsystems.groundIntake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.MotorFFGains
import com.frcteam3636.frc2025.utils.math.PIDGains
import com.frcteam3636.frc2025.utils.math.inRotationsPerSecond
import com.frcteam3636.frc2025.utils.math.*
import com.frcteam3636.frc2025.utils.math.toAngular
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage

open class PivotInputs{

    var leftMotorCurrent = Amps.zero()
    var leftMotorVelocity = RotationsPerSecond.zero()

    var rightMotorCurrent = Amps.zero()
    var rightMotorVelocity = RotationsPerSecond.zero()

    var leftPostion = Radians.zero()
    var rightPostion = Radians.zero()

    var absoluteEncoderConnected = false
}

interface PivotIO{
    fun updateInputs(inputs: PivotInputs)
    fun setMotorVoltage(voltage: Voltage)
    fun setTargetPostion(postion: Rotation2d)
}

class PivotIOReal: PivotIO{

    private val config = TalonFXConfiguration().apply {
        MotorOutput.apply {
            Inverted = InvertedValue.Clockwise_Positive
            NeutralMode = NeutralModeValue.Brake
        }

        Slot0.apply {
            pidGains = PID_GAINS
            motorFFGains = FF_GAINS
            kG = GRAVITY_GAIN
        }

        Feedback.apply {
            SensorToMechanismRatio = SENSOR_TO_MECHANISM_GEAR_RATIO
        }

        MotionMagic.apply {
            MotionMagicCruiseVelocity = PROFILE_VELOCITY.inRotationsPerSecond()
            MotionMagicAcceleration = PROFILE_ACCELERATION
            MotionMagicJerk = PROFILE_JERK
        }

        CurrentLimits.apply {
            StatorCurrentLimitEnable = true
            StatorCurrentLimit = 0.0

            SupplyCurrentLimitEnable = true
            SupplyCurrentLimit = 0.0
        }
    }

    private val leftMotor = TalonFX(CTREDeviceId.LeftPivotMotor).apply {
        configurator.apply { config }
    }

    private val rightMotor = TalonFX(CTREDeviceId.RightPivotMotor).apply {
        configurator.apply { config }
    }

    override fun updateInputs(inputs: PivotInputs) {

        inputs.leftMotorCurrent = leftMotor.supplyCurrent.value
        inputs.leftMotorVelocity = leftMotor.velocity.value

        inputs.rightMotorCurrent = rightMotor.supplyCurrent.value
        inputs.rightMotorVelocity = rightMotor.velocity.value

        inputs.leftPostion = leftMotor.position as Angle
        inputs.rightPostion = rightMotor.position as Angle

        inputs.absoluteEncoderConnected = false

    }

    override fun setTargetPostion(postion: Rotation2d) {

        val controlRequest = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations
        }

        leftMotor.setControl(controlRequest)
        rightMotor.setControl(controlRequest)
    }

    override fun setMotorVoltage(voltage: Voltage) {
        assert(voltage.inVolts() in -12.0..12.0)
        leftMotor.setVoltage(voltage,inVolts())
        rightMotor.setVoltage(voltage,inVolts())
    }

    internal companion object Constants {
        val SPOOL_RADIUS = 0.0.inches
        private val SENSOR_TO_MECHANISM_GEAR_RATIO = 0.0
        private val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
        private val FF_GAINS = MotorFFGains(0.0, 0.0, 0.0)
        private const val GRAVITY_GAIN = 0.0
        private val PROFILE_ACCELERATION = 0.0
        private const val PROFILE_JERK = 0.0
        private val PROFILE_VELOCITY = 200.inchesPerSecond.toAngular(SPOOL_RADIUS)
    }
}


