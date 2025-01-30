package com.frcteam3636.frc2025.subsystems.funnel

import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.SparkFlex
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team9432.annotation.Logged
import kotlin.math.roundToInt

@Logged
open class FunnelInputs{
    var rollerVelocity = RotationsPerSecond.zero()!!
    var rollerCurrent = Amps.zero()!!
}

interface FunnelIO{
    fun setSpeed(percent: Double)
    fun updateInputs(inputs: FunnelInputs)
}

class FunnelIOReal : FunnelIO{
    private var rampMotor = SparkFlex(
        REVMotorControllerId.RollerMotor,
        SparkLowLevel.MotorType.kBrushless
    ).apply { configure(
        SparkFlexConfig().apply{
            idleMode(IdleMode.kBrake)
            smartCurrentLimit(MOTOR_CURRENT_LIMIT.`in`(Amps).roundToInt())
        },
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters)
    }

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        rampMotor.set(percent)
    }
    override fun updateInputs(inputs: FunnelInputs) {
        inputs.rollerVelocity = RotationsPerSecond.of(rampMotor.encoder.velocity)
        inputs.rollerCurrent = Amps.of(rampMotor.outputCurrent)
    }

    internal companion object Constants{
        private val MOTOR_CURRENT_LIMIT = Amps.of(35.0)

    }
}
class FunnelIOSim : FunnelIO{
    private var motor = DCMotor.getNeoVortex(1)
    private var system = LinearSystemId.createFlywheelSystem(motor, 1.0,1.0)
    private var simMotor = FlywheelSim(system, motor, 0.0)

    override fun setSpeed(percent: Double) {
        simMotor.inputVoltage = percent * 12
    }

    override fun updateInputs(inputs: FunnelInputs) {
        simMotor.update(Robot.period)
        inputs.rollerVelocity = simMotor.angularVelocity
        simMotor.setAngularVelocity(simMotor.angularVelocityRadPerSec * 0.95)
        inputs.rollerCurrent = Amp.of(simMotor.currentDrawAmps)
    }

}

