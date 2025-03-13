package com.frcteam3636.frc2025.subsystems.climb

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.inVolts
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team9432.annotation.Logged

@Logged
open class ClimbInputs {
    var climberCurrent = Amps.zero()!!
    var current = Amps.zero()!!
    var velocity = RadiansPerSecond.zero()!!
    var position = Radians.zero()!!
}

interface ClimbIO{
    fun updateInputs(inputs: ClimbInputs)
    fun setVoltage(voltage: Voltage)
}

class ClimbIOReal: ClimbIO {

    private val climbMotor = TalonFX(CTREDeviceId.ClimbMotor)

    init {
        val config = TalonFXConfiguration().apply {
            MotorOutput.apply { NeutralMode = NeutralModeValue.Brake }
        }
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        climbMotor.configurator.apply(
            config
        )
    }

    override fun updateInputs(inputs: ClimbInputs) {
        inputs.position = climbMotor.position.value
        inputs.velocity = climbMotor.velocity.value
        inputs.current = climbMotor.torqueCurrent.value
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage.inVolts() in -12.0..12.0)
        climbMotor.setVoltage(voltage.inVolts())
    }
}

class ClimbIOSim: ClimbIO {
    private val climbSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 50.0, 60.0 ),
        DCMotor.getKrakenX60Foc(1)
        )

    override fun updateInputs(inputs: ClimbInputs) {
        climbSim.update(Robot.period)
        inputs.velocity = climbSim.angularVelocity
        inputs.current = Amps.of(climbSim.currentDrawAmps)
        climbSim.setAngularVelocity(climbSim.angularVelocityRadPerSec * 0.93)
    }

    override fun setVoltage(voltage: Voltage) {
        climbSim.setInputVoltage(voltage.inVolts())
    }
}