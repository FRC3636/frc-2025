package com.frcteam3636.frc2025.subsystems.groundIntake

import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.TalonFX
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
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
    private val leftMotor = TalonFX(CTREDeviceId.LeftPivotMotor).apply { }

    override fun setTargetPostion(postion: Rotation2d) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: PivotInputs) {
        TODO("Not yet implemented")
    }

    override fun setMotorVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }
}


