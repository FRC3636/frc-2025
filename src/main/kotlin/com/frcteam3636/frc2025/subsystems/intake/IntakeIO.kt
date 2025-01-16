package com.frcteam3636.frc2025.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.SparkFlex
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.MotorFFGains
import com.frcteam3636.frc2025.utils.math.PIDGains
import com.frcteam3636.frc2025.utils.math.motorFFGains
import com.frcteam3636.frc2025.utils.math.pidGains
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged
import kotlin.math.roundToInt

@Logged
open class IntakeInputs {
    var rollerVelocity = RotationsPerSecond.zero()!!
    var rollerCurrent = Amps.zero()!!
    var rollerPosition = Radians.zero()!!

    var armVelocity = RotationsPerSecond.zero()!!
    var armCurrent = Amps.zero()!!
    var armPosition = Radians.zero()!!
}

interface IntakeIO {
    fun setSpeed(percent: Double)
    fun pivotToAngle(position: Angle)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal: IntakeIO {

    private var intakeRollerMotor = SparkFlex(
        REVMotorControllerId.IntakeMotor,
        SparkLowLevel.MotorType.kBrushless
    ).apply {
        configure(SparkFlexConfig().apply {
            idleMode(IdleMode.kBrake)
            smartCurrentLimit(MOTOR_CURRENT_LIMIT.`in`(Amps).roundToInt())

        }, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    private var intakeArmMotor = TalonFX(CTREDeviceId.IntakeArmMotor)

    init {
        val config = TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }

            Feedback.apply {
                SensorToMechanismRatio = INTAKE_ARM_GEAR_RATIO
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

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        intakeArmMotor.configurator.apply(
            config
        )
    }

    override fun pivotToAngle(position: Angle){
        Logger.recordOutput("Intake/Arm/Position Setpoint", position)
        val control = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.`in`(Rotations)
        }
            intakeArmMotor.setControl(control)
    }

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        intakeRollerMotor.set(percent)
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.rollerVelocity = RotationsPerSecond.of(intakeRollerMotor.encoder.velocity)
        inputs.rollerCurrent = Amps.of(intakeRollerMotor.outputCurrent)
        inputs.rollerPosition = Rotations.of(intakeRollerMotor.encoder.position.mod(1.0))

        inputs.armVelocity = intakeArmMotor.velocity.value
        inputs.armCurrent = intakeArmMotor.torqueCurrent.value
        inputs.armPosition = intakeArmMotor.position.value
    }

    internal companion object Constants {
        private val MOTOR_CURRENT_LIMIT = Amps.of(35.0)
        private const val INTAKE_ARM_GEAR_RATIO = 0.0
        private val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
        private val FF_GAINS = MotorFFGains(0.0, 0.0, 0.0)
        private const val GRAVITY_GAIN = 0.0
        private const val PROFILE_ACCELERATION = 0.0
        private const val PROFILE_JERK = 0.0
        private const val PROFILE_VELOCITY = 0.0
    }

}

//class IntakeIOSim: IntakeIO {
//    intakeSimulation = Intakesimulation()
//}
