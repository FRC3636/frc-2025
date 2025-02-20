package com.frcteam3636.frc2025.subsystems.climb

import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.utils.math.amps
import com.frcteam3636.frc2025.utils.math.inVolts
import com.frcteam3636.frc2025.utils.math.volts
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Climb : Subsystem {
    private val io: ClimbIO = when (Robot.model) {
        Robot.Model.SIMULATION -> ClimbIOSim()
        Robot.Model.COMPETITION -> ClimbIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedClimbInputs()

    private var mechanism = LoggedMechanism2d(100.0, 100.0)
    private var motorAngleVisualizer =
        LoggedMechanismLigament2d("Climber motor angle", 40.0, 0.0, 5.0, Color8Bit(Color.kRed))

    init {
        mechanism.getRoot("Climber", 50.0, 50.0).apply {
            append(motorAngleVisualizer)
        }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climb", inputs)
        motorAngleVisualizer.angle += inputs.velocity.`in`(DegreesPerSecond) * Robot.period
        Logger.recordOutput("/Climber/Mechanism", mechanism)
    }

    fun up() =
        startEnd({
            io.setVoltage(12.0.volts)
        }, {
            io.setVoltage(0.0.volts)
        })!!

    fun down() =
        startEnd({
            io.setVoltage((-12.0).volts)
        }, {
            io.setVoltage(0.0.volts)
        })!!
}
