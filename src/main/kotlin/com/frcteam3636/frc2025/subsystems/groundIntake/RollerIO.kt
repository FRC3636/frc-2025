package com.frcteam3636.frc2025.subsystems.groundIntake
import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.SparkFlex
import com.frcteam3636.frc2025.SparkMax
import com.frcteam3636.frc2025.utils.math.amps
import com.frcteam3636.frc2025.utils.math.radiansPerSecond
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.units.Units.*

open class RollerInputs{
    var velocity = RotationsPerSecond.zero()
    var current = Amps.zero()
}

interface RollerIO{
    fun setSpeed(percentage: Double)
    fun updateInputs(inputs: RollerInputs)
}

class RollerIOReal : RollerIO {
    private var rollerMotor = SparkFlex(REVMotorControllerId.GroundIntakeRollerMotor, SparkLowLevel.MotorType.kBrushless).apply{
        val config = SparkFlexConfig().apply{
            idleMode(SparkBaseConfig.IdleMode.kBrake)
        }

        this.configure(config)
    }
    override fun updateInputs(inputs: RollerInputs) {
        inputs.velocity = rollerMotor.encoder.velocity.radiansPerSecond
        inputs.current = rollerMotor.outputCurrent.amps
    }

    override fun setSpeed(percentage: Double) {
        assert(percentage in -1.0..1.0)
        rollerMotor.set(percentage)
    }

}

interface RollerIOSim : RollerIO {
    // TODO()
}