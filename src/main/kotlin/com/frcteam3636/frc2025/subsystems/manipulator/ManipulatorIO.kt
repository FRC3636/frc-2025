package com.frcteam3636.frc2025.subsystems.manipulator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.range
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Current
import edu.wpi.first.wpilibj.Ultrasonic
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team9432.annotation.Logged

@Logged
open class ManipulatorInputs {
    var velocity = RotationsPerSecond.zero()!!
    var current = Amps.zero()!!

    var backUltrasonicDistance = Meters.zero()!!
    var frontUltrasonicDistance = Meters.zero()!!
}

interface ManipulatorIO {
    fun setSpeed(percent: Double)
    fun setCurrent(current: Current)
    fun updateInputs(inputs: ManipulatorInputs)
}

class ManipulatorIOReal : ManipulatorIO {
    private var manipulatorMotor = TalonFX(CTREDeviceId.ManipulatorMotor)
    private var backUltrasonic = Ultrasonic(BACK_ULTRASONIC_PING_CHANNEL, BACK_ULTRASONIC_ECHO_CHANNEL)
    private var frontUltrasonic = Ultrasonic(FRONT_ULTRASONIC_PING_CHANNEL, FRONT_ULTRASONIC_ECHO_CHANNEL)

    init {
        val config = TalonFXConfiguration().apply {
            MotorOutput.apply { NeutralMode = NeutralModeValue.Brake }
        }
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        manipulatorMotor.configurator.apply(
            config
        )
    }

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        manipulatorMotor.set(percent)
    }

    override fun setCurrent(current: Current) {
        val controlRequest = TorqueCurrentFOC(current)
        manipulatorMotor.setControl(controlRequest)
    }

    override fun updateInputs(inputs: ManipulatorInputs) {
        inputs.velocity = manipulatorMotor.velocity.value
        inputs.current = manipulatorMotor.torqueCurrent.value

        inputs.backUltrasonicDistance = backUltrasonic.range
        inputs.frontUltrasonicDistance = frontUltrasonic.range
    }

    internal companion object Constants {
        private const val BACK_ULTRASONIC_PING_CHANNEL = 1
        private const val BACK_ULTRASONIC_ECHO_CHANNEL = 2
        private const val FRONT_ULTRASONIC_PING_CHANNEL = 3
        private const val FRONT_ULTRASONIC_ECHO_CHANNEL = 4
    }
}

class ManipulatorIOSim : ManipulatorIO {
    private var motor = DCMotor.getKrakenX60Foc(1)
    private var system = LinearSystemId.createFlywheelSystem(motor, 1.0, 1.0)
    private var simMotor = FlywheelSim(system, motor, 0.0)

    override fun setSpeed(percent: Double) {
        simMotor.inputVoltage = percent * 12
    }

    override fun setCurrent(current: Current) {
        TODO("Not implemented yet")
    }

    override fun updateInputs(inputs: ManipulatorInputs) {
        simMotor.update(Robot.period)
        inputs.velocity = simMotor.angularVelocity
        simMotor.setAngularVelocity(simMotor.angularVelocityRadPerSec * 0.95)
        inputs.current = Amps.of(simMotor.currentDrawAmps)
    }

}
