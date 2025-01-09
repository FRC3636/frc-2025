package com.frcteam3636.frc2025.subsystems.drivetrain

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TorqueCurrentConfigs
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.frcteam3636.frc2025.*
import com.frcteam3636.frc2025.utils.math.*
import com.frcteam3636.frc2025.utils.swerve.speed
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import org.littletonrobotics.junction.Logger
import kotlin.math.roundToInt

interface SwerveModule {
    // The current "state" of the swerve module.
    //
    // This is essentially the velocity of the wheel,
    // and includes both the speed and the angle
    // in which the module is currently traveling.
    val state: SwerveModuleState

    // The desired state of the module.
    //
    // This is the wheel velocity that we're trying to get to.
    var desiredState: SwerveModuleState

    // The measured position of the module.
    //
    // This is a vector with direction equal to the current angle of the module,
    // and magnitude equal to the total signed distance traveled by the wheel.
    val position: SwerveModulePosition

    fun periodic() {}
}

class MAXSwerveModule(
    private val drivingMotor: DrivingMotor, turningId: REVMotorControllerId, private val chassisAngle: Rotation2d
) : SwerveModule {
    private val turningSpark = SparkMax(turningId, SparkLowLevel.MotorType.kBrushless).apply {
        configure(SparkMaxConfig().apply {
            idleMode(IdleMode.kBrake)
            smartCurrentLimit(TURNING_CURRENT_LIMIT.roundToInt())

            absoluteEncoder.apply {
                inverted(true)
                positionConversionFactor(TAU)
                velocityConversionFactor(TAU / 60)
            }

            closedLoop.apply {
                pid(TURNING_PID_GAINS.p, TURNING_PID_GAINS.i, TURNING_PID_GAINS.d)
                feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                positionWrappingEnabled(true)
                positionWrappingMinInput(0.0)
                positionWrappingMaxInput(TAU)
            }
        }, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    // whereas the turning encoder must be absolute so that
    // we know where the wheel is pointing
    private val turningEncoder = turningSpark.getAbsoluteEncoder()


    private val turningPIDController = turningSpark.closedLoopController

    override val state: SwerveModuleState
        get() = SwerveModuleState(
            drivingMotor.velocity.`in`(MetersPerSecond), Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
        )

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
            drivingMotor.position, Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
        )

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, -chassisAngle)
        get() = SwerveModuleState(field.speedMetersPerSecond, field.angle + chassisAngle)
        set(value) {
            val corrected = SwerveModuleState(value.speedMetersPerSecond, value.angle - chassisAngle)
            // optimize the state to avoid rotating more than 90 degrees
            corrected.optimize(
                Rotation2d.fromRadians(turningEncoder.position)
            )

            drivingMotor.velocity = corrected.speed

            turningPIDController.setReference(
                corrected.angle.radians, SparkBase.ControlType.kPosition
            )


            field = corrected
        }
}

interface DrivingMotor {
    val position: Distance
    var velocity: LinearVelocity
}

class DrivingTalon(id: CTREDeviceId) : DrivingMotor {

    private val inner = TalonFX(id).apply {
        configurator.apply(Slot0Configs().apply {
            pidGains = DRIVING_PID_GAINS_TALON
            motorFFGains = DRIVING_FF_GAINS_TALON
        })
        // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html#stator-and-supply-current-limits
        configurator.apply(
            TorqueCurrentConfigs().apply {
                withPeakForwardTorqueCurrent(DRIVING_CURRENT_LIMIT)
                withPeakReverseTorqueCurrent(-DRIVING_CURRENT_LIMIT)
            })

    }

    init {
        Robot.statusSignals[id.name] = inner.version
    }

    override val position: Distance
        get() = Meters.of(inner.position.value.`in`(Rotations) * DRIVING_GEAR_RATIO_TALON * WHEEL_CIRCUMFERENCE.`in`(Meters))

    override var velocity: LinearVelocity
        get() = MetersPerSecond.of(inner.velocity.value.`in`(RotationsPerSecond) * DRIVING_GEAR_RATIO_TALON * WHEEL_CIRCUMFERENCE.`in`(Meters))
        set(value) {
            inner.setControl(VelocityTorqueCurrentFOC(value.`in`(MetersPerSecond) / DRIVING_GEAR_RATIO_TALON / WHEEL_CIRCUMFERENCE.`in`(Meters)))
        }
}

class DrivingSparkMAX(val id: REVMotorControllerId) : DrivingMotor {
    private val inner = SparkMax(id, SparkLowLevel.MotorType.kBrushless).apply {
        val innerConfig = SparkMaxConfig().apply {
            idleMode(IdleMode.kBrake)
            smartCurrentLimit(DRIVING_CURRENT_LIMIT.`in`(Amps).toInt())
            inverted(true)

            encoder.apply {
                positionConversionFactor(WHEEL_CIRCUMFERENCE.`in`(Meters) / DRIVING_GEAR_RATIO_NEO)
                velocityConversionFactor(WHEEL_CIRCUMFERENCE.`in`(Meters) / DRIVING_GEAR_RATIO_NEO / 60)
            }

            closedLoop.apply {
                pid(DRIVING_PID_GAINS_NEO.p, DRIVING_PID_GAINS_NEO.i, DRIVING_PID_GAINS_NEO.d)
                velocityFF(DRIVING_FF_GAINS_NEO.v)
                feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            }
        }
        configure(innerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override val position: Distance
        get() = Meters.of(inner.encoder.position)

    override var velocity: LinearVelocity
        get() = MetersPerSecond.of(inner.encoder.velocity)
        set(value) {
            Logger.recordOutput("/Drivetrain/$id/OutputVel", value)
            inner.closedLoopController.setReference(value.`in`(MetersPerSecond), SparkBase.ControlType.kVelocity)
        }
}
//
//class SimSwerveModule : SwerveModule {
//
//    // TODO: figure out what the moment of inertia actually is and if it even matters
////    private val turningMotor = DCMotorSim(DCMotor.getNeo550(1), TAU, 0.0001)
//    private val turningMotor = DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.0, 1.0), )
//    private val drivingMotor = DCMotorSim(DCMotor.getKrakenX60(1), 6.75, 0.0025)
//
//    private val drivingFeedforward = SimpleMotorFeedforward(MotorFFGains(v = 3.33))
//    private val drivingFeedback = PIDController(PIDGains(0.06))
//
//    private val turningFeedback = PIDController(PIDGains(p = 2.0)).apply { enableContinuousInput(0.0, TAU) }
//
//    override val state: SwerveModuleState
//        get() = SwerveModuleState(
//            drivingMotor.angularVelocityRadPerSec * WHEEL_RADIUS,
//            Rotation2d.fromRadians(turningMotor.angularPositionRad)
//        )
//
//    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())
//        set(value) {
//            field = SwerveModuleState.optimize(value, state.angle)
//        }
//
//    override val position: SwerveModulePosition
//        get() = SwerveModulePosition(
//            drivingMotor.angularPositionRad * WHEEL_RADIUS, Rotation2d.fromRadians(turningMotor.angularPositionRad)
//        )
//
//    override fun periodic() {
//        turningMotor.update(Robot.period)
//        drivingMotor.update(Robot.period)
//
//        // Set the new input voltages
//        turningMotor.setInputVoltage(
//            turningFeedback.calculate(state.angle.radians, desiredState.angle.radians)
//        )
//        drivingMotor.setInputVoltage(
//            drivingFeedforward.calculate(desiredState.speedMetersPerSecond) + drivingFeedback.calculate(
//                state.speedMetersPerSecond, desiredState.speedMetersPerSecond
//            )
//        )
//    }
//}

// take the known wheel diameter, divide it by two to get the radius, then get the
// circumference
internal val WHEEL_RADIUS = Inches.of(3.0).div(2.0)
internal val WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * TAU

internal val NEO_FREE_SPEED = RPM.of(5676.0)

private const val DRIVING_MOTOR_PINION_TEETH = 14

internal const val DRIVING_GEAR_RATIO_TALON = 1.0 / 3.56
internal const val DRIVING_GEAR_RATIO_NEO = (45.0 * 22.0) / (DRIVING_MOTOR_PINION_TEETH * 15.0)

internal val NEO_DRIVING_FREE_SPEED = MetersPerSecond.of((NEO_FREE_SPEED.`in`(RotationsPerSecond) * WHEEL_CIRCUMFERENCE.`in`(Meters)) / DRIVING_GEAR_RATIO_NEO)

internal val DRIVING_PID_GAINS_TALON: PIDGains = PIDGains(4.0, 0.0, 0.1)
internal val DRIVING_PID_GAINS_NEO: PIDGains = PIDGains(0.04, 0.0, 0.0)
internal val DRIVING_FF_GAINS_TALON: MotorFFGains = MotorFFGains(5.75, 0.0)
internal val DRIVING_FF_GAINS_NEO: MotorFFGains =
    MotorFFGains(0.0, 1 / NEO_DRIVING_FREE_SPEED.`in`(MetersPerSecond), 0.0) // TODO: ensure this is right

internal val TURNING_PID_GAINS: PIDGains = PIDGains(1.7, 0.0, 0.125)
internal val DRIVING_CURRENT_LIMIT = Amps.of(35.0)
internal const val TURNING_CURRENT_LIMIT = 20.0
