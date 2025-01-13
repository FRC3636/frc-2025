package com.frcteam3636.frc2025.subsystems.elevator

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.CANcoder
import com.frcteam3636.frc2025.utils.math.MotorFFGains
import com.frcteam3636.frc2025.utils.math.PIDGains
import com.frcteam3636.frc2025.utils.math.motorFFGains
import com.frcteam3636.frc2025.utils.math.pidGains
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged

open class ElevatorInputs {
    var height = Meters.zero()!!
    var rightCurrent = Volts.zero()!!
    var leftCurrent = Volts.zero()!!
    var velocity = MetersPerSecond.zero()!!
}

interface ElevatorIO{
    fun updateInputs(inputs: ElevatorInputs)

    fun runToHeight(height: Distance)

    fun setVoltage(volts: Voltage)

}

class ElevatorIOReal: ElevatorIO {

    private val encoder = CANcoder(CTREDeviceId.ElevatorEncoder).apply {
        val config = CANcoderConfiguration().apply {
            MagnetSensor.apply {
                withAbsoluteSensorDiscontinuityPoint(Rotations.one())
                SensorDirection = SensorDirectionValue.Clockwise_Positive
            }
        }
        configurator.apply(config)
    }

    val config = TalonFXConfiguration().apply {
        MotorOutput.apply {
            NeutralMode = NeutralModeValue.Brake
        }

        Feedback.apply {
            SensorToMechanismRatio = GEAR_RATIO
            FeedbackRemoteSensorID = CTREDeviceId.ElevatorEncoder.num
            FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
        }

        Slot0.apply {
            pidGains = PID_GAINS
            motorFFGains = FF_GAINS
            kG = GRAVITY_GAIN
        }

        MotionMagic.apply {
            MotionMagicCruiseVelocity = PROFILE_VELOCITY
            MotionMagicAcceleration = PROFILE_ACCELERATION
            MotionMagicJerk = PROFILE_JERK
        }
    }

    private val rightElevatorMotor = TalonFX(CTREDeviceId.ElevatorMotor).apply {
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        configurator.apply(config)
    }

    private val leftElevatorMotor = TalonFX(CTREDeviceId.ElevatorMotor).apply {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        configurator.apply(config)
    }

    override fun updateInputs(inputs: ElevatorInputs) {
        inputs.height = (METERS_PER_ROTATION * encoder.position.value) as Distance
        inputs.velocity = (METERS_PER_ROTATION * encoder.velocity.value) as LinearVelocity
        inputs.rightCurrent = rightElevatorMotor.motorVoltage.value
        inputs.leftCurrent = leftElevatorMotor.motorVoltage.value
    }

    override fun runToHeight(height: Distance) {
        Logger.recordOutput("Elevator/Height Setpoint", height)
        var desiredMotorAngle = (height / METERS_PER_ROTATION) as Angle
        var controlRequest = MotionMagicTorqueCurrentFOC(desiredMotorAngle)
        rightElevatorMotor.setControl(controlRequest)
        leftElevatorMotor.setControl(controlRequest)
    }

    override fun setVoltage(volts: Voltage){
        assert(volts in Volts.of(-12.0)..Volts.of(12.0))
        val controlRequest = VoltageOut(volts.`in`(Volts))
        rightElevatorMotor.setControl(controlRequest)
        leftElevatorMotor.setControl(controlRequest)
    }

    internal companion object Constants {
        val METERS_PER_ROTATION = Meters.per(Rotation).of(0.0)!!
        private const val GEAR_RATIO = 0.0
        val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
        val FF_GAINS = MotorFFGains(0.0, 0.0, 0.0)
        private const val GRAVITY_GAIN = 0.0
        private const val PROFILE_ACCELERATION = 0.0
        private const val PROFILE_JERK = 0.0
        private const val PROFILE_VELOCITY = 0.0
    }

}