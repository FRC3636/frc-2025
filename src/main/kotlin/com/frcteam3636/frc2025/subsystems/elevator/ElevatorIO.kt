package com.frcteam3636.frc2025.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.*
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged


@Logged
open class ElevatorInputs {
    var height = 0.meters
    var rightCurrent = 0.amps
    var leftCurrent = 0.amps
    var velocity = 0.metersPerSecond
}

interface ElevatorIO {
    fun updateInputs(inputs: ElevatorInputs)

    fun runToHeight(height: Distance)
    fun runToHeightWithOverride(height: Distance, velocity: AngularVelocity, acceleration: AngularAcceleration)

    fun setVoltage(volts: Voltage)

    fun setEncoderPosition(position: Distance)

    fun setBrakeMode(enabled: Boolean) {}

    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf()
    }
}

class ElevatorIOReal : ElevatorIO {

//    private val encoder = CANcoder(CTREDeviceId.ElevatorEncoder).apply {
//        val config = CANcoderConfiguration().apply {
//            MagnetSensor.apply {
//                withAbsoluteSensorDiscontinuityPoint(Rotations.one())
//                SensorDirection = SensorDirectionValue.Clockwise_Positive
//            }
//        }
//        configurator.apply(config)
//    }

    private val leftConfig = TalonFXConfiguration().apply {
        MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
    }
    private val rightConfig = TalonFXConfiguration().apply {
        MotorOutput.Inverted = InvertedValue.Clockwise_Positive
    }

    private val rightElevatorMotor = TalonFX(CTREDeviceId.RightElevatorMotor)
    private val leftElevatorMotor = TalonFX(CTREDeviceId.LeftElevatorMotor)

    init {
        configure {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }

            Slot0.apply {
                pidGains = PID_GAINS
//                motorFFGains = FF_GAINS
//                kG = GRAVITY_GAIN
            }

            Feedback.apply {
                SensorToMechanismRatio = ROTOR_TO_MECHANISM_GEAR_RATIO
            }

            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_VELOCITY.inRotationsPerSecond()
                MotionMagicAcceleration = PROFILE_ACCELERATION
                MotionMagicJerk = PROFILE_JERK
            }
//
//            CurrentLimits.apply {
//                StatorCurrentLimitEnable = true
//                StatorCurrentLimit = 37.0
//
//                SupplyCurrentLimitEnable = true
//                SupplyCurrentLimit = 20.0
//            }
        }
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, leftElevatorMotor.position, leftElevatorMotor.velocity, rightElevatorMotor.supplyCurrent, leftElevatorMotor.supplyCurrent)
        leftElevatorMotor.optimizeBusUtilization()
        rightElevatorMotor.optimizeBusUtilization()
    }

    override fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return mutableListOf(leftElevatorMotor.getPosition(false), leftElevatorMotor.getSupplyCurrent(false), leftElevatorMotor.getVelocity(false), rightElevatorMotor.getSupplyCurrent(false))
    }

    private inline fun configure(config: TalonFXConfiguration.() -> Unit) {
        leftConfig.apply(config)
        rightConfig.apply(config)

        leftElevatorMotor.configurator.apply(leftConfig)
        rightElevatorMotor.configurator.apply(rightConfig)
    }

    override fun updateInputs(inputs: ElevatorInputs) {
        inputs.height = leftElevatorMotor.getPosition(false).value.toLinear(SPOOL_RADIUS)
        inputs.velocity = leftElevatorMotor.getVelocity(false).value.toLinear(SPOOL_RADIUS)
        inputs.rightCurrent = rightElevatorMotor.getSupplyCurrent(false).value
        inputs.leftCurrent = leftElevatorMotor.getSupplyCurrent(false).value
    }

    override fun runToHeight(height: Distance) {
        Logger.recordOutput("Elevator/Height Setpoint", height)
        val desiredMotorAngle = height.toAngular(SPOOL_RADIUS)
        val controlRequest = MotionMagicVoltage(desiredMotorAngle)
        rightElevatorMotor.setControl(controlRequest)
        leftElevatorMotor.setControl(controlRequest)
    }

    override fun runToHeightWithOverride(
        height: Distance,
        velocity: AngularVelocity,
        acceleration: AngularAcceleration
    ) {
        Logger.recordOutput("Elevator/Height Setpoint", height)
        val desiredMotorAngle = height.toAngular(SPOOL_RADIUS)
        val controlRequest = DynamicMotionMagicVoltage(
            desiredMotorAngle.inRotations(),
            velocity.inRotationsPerSecond(),
            acceleration.baseUnitMagnitude(),
            0.0
        )
        rightElevatorMotor.setControl(controlRequest)
        leftElevatorMotor.setControl(controlRequest)
    }

    override fun setVoltage(volts: Voltage) {
        assert(volts in (-12).volts..12.volts)
        val controlRequest = VoltageOut(volts.inVolts())
        rightElevatorMotor.setControl(controlRequest)
        leftElevatorMotor.setControl(controlRequest)
    }

    override fun setEncoderPosition(position: Distance) {
        assert(position in 0.meters..1.5.meters)
        rightElevatorMotor.setPosition(position.toAngular(SPOOL_RADIUS))
        leftElevatorMotor.setPosition(position.toAngular(SPOOL_RADIUS))
    }

    override fun setBrakeMode(enabled: Boolean) {
        val neutralMode = if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast
        configure {
            MotorOutput.NeutralMode = neutralMode
        }
    }

    internal companion object Constants {
        // https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A37%2C%22u%22%3A%22A%22%7D&efficiency=85.4&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A12%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%2A%22%7D&ratio=%7B%22magnitude%22%3A8%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.54%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A48%2C%22u%22%3A%22in%22%7D
        private const val ROTOR_TO_MECHANISM_GEAR_RATIO = 8.0
        val SPOOL_RADIUS = 0.77.inches

        //        private val DISTANCE_PER_TURN = Meters.per(Radian).of(SPOOL_RADIUS.meters)
        private val PID_GAINS = PIDGains(30.0, 0.0, 0.0)
        private val FF_GAINS = MotorFFGains(0.039214, 1.0233, 0.025904)
        private const val GRAVITY_GAIN = 0.27592
        private const val PROFILE_ACCELERATION = 50.0 // TODO: Increase to good setting
        private const val PROFILE_JERK = 0.0
        private val PROFILE_VELOCITY = 350.inchesPerSecond.toAngular(SPOOL_RADIUS)
    }

}

class ElevatorIOSim : ElevatorIO {
    // Simulation classes help us simulate what's going on, including gravity.
    private var motor: DCMotor = DCMotor.getKrakenX60Foc(2)

    private val elevatorSim: ElevatorSim = ElevatorSim(
        motor,
        GEAR_RATIO,
        CARRIAGE_MASS,
        DRUM_RADIUS,
        MIN_HEIGHT,
        MAX_HEIGHT,
        true,
        0.01
    )

    private var controller = ProfiledPIDController(
        PID_GAINS.p,
        PID_GAINS.i,
        PID_GAINS.d,
        TRAPEZOID_CONSTRAINTS
    )

    private var feedforward = ElevatorFeedforward(
        FF_GAINS.s,
        FF_GAINS.v,
        FF_GAINS.a
    )

    override fun updateInputs(inputs: ElevatorInputs) {
        elevatorSim.update(Robot.period)
        inputs.height = elevatorSim.positionMeters.meters
        inputs.velocity = elevatorSim.velocityMetersPerSecond.metersPerSecond
        inputs.leftCurrent = elevatorSim.currentDrawAmps.amps
        inputs.rightCurrent = elevatorSim.currentDrawAmps.amps
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.currentDrawAmps))
    }

    override fun runToHeight(height: Distance) {
        val pidOutput = controller.calculate(elevatorSim.positionMeters)
        val feedforwardOutput = feedforward.calculate(controller.setpoint.velocity)
        elevatorSim.setInputVoltage(pidOutput + feedforwardOutput)
    }

    override fun runToHeightWithOverride(
        height: Distance,
        velocity: AngularVelocity,
        acceleration: AngularAcceleration
    ) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(volts: Voltage) {
        elevatorSim.setInputVoltage(volts.inVolts())
        Logger.recordOutput("/Elevator/OutVolt", volts)
    }

    override fun setEncoderPosition(position: Distance) {
        TODO("Not yet implemented")
    }

    internal companion object Constants {
        private const val GEAR_RATIO = 1.0
        private const val CARRIAGE_MASS = 1.62519701308126
        private const val MIN_HEIGHT = 0.254000
        private const val MAX_HEIGHT = 2.298700
        private const val DRUM_RADIUS = 0.028575 / 2
        val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
        val FF_GAINS = MotorFFGains(0.0, 2.77, 0.05)
        var TRAPEZOID_CONSTRAINTS = TrapezoidProfile.Constraints(4.16, 8.2119724)
    }
}
