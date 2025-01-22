package com.frcteam3636.frc2025.subsystems.elevator

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.sim.TalonFXSimState
import com.frcteam3636.frc2025.CANcoder
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.Robot.controller
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.elevator.ElevatorIOReal.Constants
import com.frcteam3636.frc2025.utils.math.MotorFFGains
import com.frcteam3636.frc2025.utils.math.PIDGains
import com.frcteam3636.frc2025.utils.math.motorFFGains
import com.frcteam3636.frc2025.utils.math.pidGains
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import edu.wpi.first.wpilibj.simulation.*
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
        inputs.height = (DISTANCE_PER_TURN * encoder.position.value) as Distance
        inputs.velocity = (DISTANCE_PER_TURN * encoder.velocity.value) as LinearVelocity
        inputs.rightCurrent = rightElevatorMotor.motorVoltage.value
        inputs.leftCurrent = leftElevatorMotor.motorVoltage.value
    }

    override fun runToHeight(height: Distance) {
        Logger.recordOutput("Elevator/Height Setpoint", height)
        var desiredMotorAngle = (height / DISTANCE_PER_TURN) as Angle
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
        val DISTANCE_PER_TURN = Meters.per(Rotation).of(0.0)!!
        private const val GEAR_RATIO = 0.0
        val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
        val FF_GAINS = MotorFFGains(0.0, 0.0, 0.0)
        private const val GRAVITY_GAIN = 0.0
        private const val PROFILE_ACCELERATION = 0.0
        private const val PROFILE_JERK = 0.0
        private const val PROFILE_VELOCITY = 0.0
    }

}

class ElevatorIOSim: ElevatorIO {
    // Simulation classes help us simulate what's going on, including gravity.
    var motor = DCMotor.getKrakenX60Foc(2)
    var motorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(motor,0.0000763789,25.0),
        motor,
        0.0
    )

    private val elevatorSim: ElevatorSim = ElevatorSim(
        motor,
        GEAR_RATIO,
        CARRIAGE_MASS,
        DRUM_RADIUS,
        MIN_HEIGHT,
        MAX_HEIGHT,
        true,
        0.01,
        0.0
    )

    var controller = ProfiledPIDController(
        PID_GAINS.p,
        PID_GAINS.i,
        PID_GAINS.d,
        TRAPEZOID_CONSTRAINTS
    )

    var feedforward = ElevatorFeedforward(
        FF_GAINS.s,
        FF_GAINS.v,
        FF_GAINS.a
    )

    var encoder = Encoder(0,1)
    var encoderSim = EncoderSim(encoder)

    override fun updateInputs(inputs: ElevatorInputs) {
        elevatorSim.setInput(motorSim.angularVelocity * RobotController.getBatteryVoltage())
        elevatorSim.update(.020)
        encoderSim.setDistance(elevatorSim.positionMeters)
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.currentDrawAmps))
    }

    override fun runToHeight(height: Distance) {
        var pidOutput = controller.calculate(encoder.distance)
        var feedforwardOutput = feedforward.calculate(controller.setpoint.velocity)
        motor.setVoltage(pidOutput + feedforwardOutput)
    }

    override fun setVoltage(volts: Voltage) {
        elevatorSim.setInputVoltage(volts.`in`(Volts))
        Logger.recordOutput("/Elevator/OutVolt", volts)
    }

    internal companion object Constants {
        val DISTANCE_PER_TURN = Meters.per(Rotation).of(0.0)!!
        private const val GEAR_RATIO = 0.0
        private const val CARRIAGE_MASS = 1.62519701308126
        private const val MIN_HEIGHT = 0.254000
        private const val MAX_HEIGHT = 2.298700
        private const val DRUM_RADIUS = 0.028575/2
        val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
        val FF_GAINS = MotorFFGains(0.0, 0.0, 0.0)
        var TRAPEZOID_CONSTRAINTS = TrapezoidProfile.Constraints(1.0, 1.0)
        private const val GRAVITY_GAIN = 0.0
        private const val PROFILE_ACCELERATION = 0.0
        private const val PROFILE_JERK = 0.0
        private const val PROFILE_VELOCITY = 0.0
    }
}