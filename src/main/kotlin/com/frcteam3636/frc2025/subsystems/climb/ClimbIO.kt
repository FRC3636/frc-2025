package com.frcteam3636.frc2025.subsystems.climb

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.frcteam3636.frc2025.CANcoder
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.toAngular
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class ClimbInputs {
    var current = Amps.zero()!!
    var velocity = RadiansPerSecond.zero()!!
    var position = Radians.zero()!!
}

interface ClimbIO{
    fun updateInputs(inputs: ClimbInputs)

    fun turnToAngle(angle: Angle)

    fun setVoltage(volts: Voltage)

    fun setPosition(position: Distance)
}

class ClimbIOReal: ClimbIO {

    private val encoder = CANcoder(CTREDeviceId.ClimbEncoder).apply {
        val config = CANcoderConfiguration().apply {
            MagnetSensor.apply {
                withAbsoluteSensorDiscontinuityPoint(Rotations.one())
                SensorDirection = SensorDirectionValue.Clockwise_Positive
            }
        }
        configurator.apply(config)
    }

    private val climbMotor = TalonFX(CTREDeviceId.ClimbMotor)

    override fun updateInputs(inputs: ClimbInputs) {
        inputs.position = encoder.position.value
        inputs.velocity = encoder.velocity.value
        inputs.current = climbMotor.torqueCurrent.value
    }

    override fun turnToAngle(angle: Angle) {
        Logger.recordOutput("Shooter/Pivot/Position Setpoint", angle)

        val control = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = angle.`in`(Rotations)
        }
        climbMotor.setControl(control)
    }

    override fun setVoltage(volts: Voltage) {
        assert(volts in Volts.of(-12.0)..Volts.of(12.0))
        val control = VoltageOut(volts.`in`(Volts))
        climbMotor.setControl(control)
    }

    override fun setPosition(position: Distance) {
        climbMotor.setPosition(position.toAngular(CLIMBER_RADIUS))
    }

    internal companion object Constants {
        private val CLIMBER_RADIUS = Inches.of(5.0) //placeholder
    }
}