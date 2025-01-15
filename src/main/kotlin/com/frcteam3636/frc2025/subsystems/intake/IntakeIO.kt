package com.frcteam3636.frc2025.subsystems.intake

import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.SparkFlex
import com.frcteam3636.frc2025.TalonFX
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.units.Units.*
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
    fun setIsLowered(boolean: Boolean)
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

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        intakeRollerMotor.set(percent)
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.rollerVelocity = RotationsPerSecond.of(intakeRollerMotor.encoder.velocity)
        inputs.current = Amps.of(intakeRollerMotor.outputCurrent)
        inputs.position = Rotations.of(intakeRollerMotor.encoder.position.mod(1.0))
    }
}

//class IntakeIOSim: IntakeIO {
//    intakeSimulation = Intakesimulation()
//}

internal val MOTOR_CURRENT_LIMIT = Amps.of(35.0)