package com.frcteam3636.frc2025.subsystems.climb

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.sim.TalonFXSimState
import com.frcteam3636.frc2025.CANcoder
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.inVolts
import com.frcteam3636.frc2025.utils.math.rotations
import com.frcteam3636.frc2025.utils.math.toAngular
import com.revrobotics.spark.SparkRelativeEncoder
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Measure
import com.frcteam3636.frc2025.utils.math.volts
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class ClimbInputs {
    var climberCurrent = Amps.zero()!!
//    var velocity = RadiansPerSecond.zero()!!
//    var position = Radians.zero()!!
    var current = Amps.zero()!!
    var velocity = RadiansPerSecond.zero()!!
    var position = Radians.zero()!!
}

interface ClimbIO{
    fun updateInputs(inputs: ClimbInputs)
    fun setSpeed(percent: Double)
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

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        climbMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage.inVolts() in -12.0..12.0)
        climbMotor.setVoltage(voltage.inVolts())
    }
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

    override fun setSpeed(percent: Double) {}

    override fun setVoltage(voltage: Voltage) {
        climbSim.setInputVoltage(voltage.inVolts())
    }
}