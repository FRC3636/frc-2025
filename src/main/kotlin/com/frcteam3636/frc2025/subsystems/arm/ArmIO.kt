package com.frcteam3636.frc2025.subsystems.arm

import com.ctre.phoenix6.configs.Slot0Configs
import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.SparkFlex
import com.frcteam3636.frc2025.utils.math.TAU
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SmartMotionConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.LinearVelocity

open class ArmInputs{
    var pivotLeftCurrent = Amps.zero()!!
    var pivotLeftPostion = Meters.zero()!!
    var pivotLeftVelocity = MetersPerSecond.zero()!!

    var pivotRightCurrent = Amps.zero()!!
    var pivotRightPostion = Meters.zero()!!
    var pivotRightVelocity = MetersPerSecond.zero()!!


    var RollerCurrent = Amps.zero()!!
    var RollerVelocity = MetersPerSecond.zero()!!



}

interface armIO{
    fun setTargetPivotPostion(angle: Angle)
    fun setRollerSpeed(percent: Double)
    fun updateInputs(inputs: ArmInputs)
}

class ArmIOReal : armIO{
    private var pivotRightMotor = SparkFlex(
        REVMotorControllerId.PivotRightMotor,
        SparkLowLevel.MotorType.kBrushless
    ).apply {
        configure(SparkFlexConfig().apply{
            idleMode(SparkBaseConfig.IdleMode.kBrake)
            smartCurrentLimit(20)

            absoluteEncoder.apply{
                inverted(true)//fix this once the arm is assembled
            }

            closedLoop.apply{
                pid(PID_GAINS.p, PID_GAIN.i,PID_GAIN.d)
                feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)


                velocityFF()

            }



        }, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)s
    }
    private val
    private var pivotLeftMotor = SparkFlex(
        REVMotorControllerId.PivotLeftMotor,
        SparkLowLevel.MotorType.kBrushless
    )
    private var pivotRollerMotor = SparkFlex(
        REVMotorControllerId.PivotRollerMotor,
        SparkLowLevel.MotorType.kBrushless
    )

    override fun updateInputs(inputs: ArmInputs) {
        inputs.rollerCurrent = Amps.of(pivotRollerMotor.outputCurrent)
        inputs.pivotLeftCurrent = Amps.of(pivotLeftMotor.outputCurrent)
        inputs.pivotRightCurrent = Amps.of(pivotRightMotor.outputCurrent)

        inputs.pivotLeftVelocity = LinearVelocity.ofBaseUnits(pivotLeftMotor.encoder.velocity, Units.MetersPerSecond)
        inputs.pivotRightVelocity = LinearVelocity.ofBaseUnits(pivotRightMotor.encoder.velocity, Units.MetersPerSecond)
        inputs.rollerVelocity = LinearVelocity.ofBaseUnits(pivotRollerMotor.encoder.velocity, Units.MetersPerSecond)

        inputs.pivotLeftPostion = (DISTANCE_PER_ROTATION * pivotLeftMotor.encoder.position)
        inputs.pivotRightPostion = (DISTANCE_PER_ROTATION * pivotRightMotor.encoder.position)
    }


}