package com.frcteam3636.frc2025.subsystems.climb

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.volts
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class ClimbInputs {
    var climberCurrent = Amps.zero()!!
}

interface ClimbIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: ClimbInputs)
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

    override fun updateInputs(inputs: ClimbInputs) {
        inputs.climberCurrent = climbMotor.supplyCurrent.value
    }
}