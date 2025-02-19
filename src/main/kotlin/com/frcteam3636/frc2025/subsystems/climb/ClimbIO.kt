package com.frcteam3636.frc2025.subsystems.climb

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.rotations
import com.frcteam3636.frc2025.utils.math.toAngular
import com.frcteam3636.frc2025.utils.math.volts
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class ClimbInputs {
    var climberCurrent = Amps.zero()!!
//    var velocity = RadiansPerSecond.zero()!!
//    var position = Radians.zero()!!
}

interface ClimbIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: ClimbInputs)
//    fun setPosition(position: Distance)
}

class ClimbIOReal : ClimbIO {
    private var climbMotor = TalonFX(CTREDeviceId.ClimbMotor).apply {
        configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                }
            }
        )
    }

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        climbMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage.volts in -12.0..12.0)
        climbMotor.setVoltage(voltage.volts)
    }

//    override fun turnToAngle(angle: Angle) {
//        Logger.recordOutput("Shooter/Pivot/Position Setpoint", angle)
//
//        val control = MotionMagicTorqueCurrentFOC(0.0).apply {
//            Slot = 0
//            Position = angle.rotations
//        }
//        climbMotor.setControl(control)
//    }

//    override fun setPosition(position: Distance) {
//        climbMotor.setPosition(position.toAngular(CLIMBER_RADIUS))
//    }

//    internal companion object Constants {
//        private val CLIMBER_RADIUS = Inches.of(5.0) //placeholder
//    }

    override fun updateInputs(inputs: ClimbInputs) {
        inputs.climberCurrent = climbMotor.supplyCurrent.value
    }
}