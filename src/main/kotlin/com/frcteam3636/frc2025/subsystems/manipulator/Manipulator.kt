package com.frcteam3636.frc2025.subsystems.manipulator

import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.utils.math.degreesPerSecond
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Manipulator : Subsystem {
    private val io: ManipulatorIO = when (Robot.model) {
        Robot.Model.SIMULATION -> ManipulatorIOSim()
        Robot.Model.COMPETITION -> ManipulatorIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedManipulatorInputs()

    private var mechanism = LoggedMechanism2d(100.0, 100.0)
    private var motorAngleVisualizer =
        LoggedMechanismLigament2d("Manipulator Motor Angle", 40.0, 0.0, 5.0, Color8Bit(Color.kRed))

    init {
        mechanism.getRoot("Manipulator", 50.0, 50.0).apply {
            append(motorAngleVisualizer)
        }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Manipulator", inputs)

        motorAngleVisualizer.angle += inputs.velocity.degreesPerSecond * Robot.period
        Logger.recordOutput("/Manipulator/Mechanism", mechanism)
    }

    private val coralInIntakeBack get() = inputs.backUltrasonicDistance < Meters.zero()
    private val coralInIntakeFront get() = inputs.frontUltrasonicDistance < Meters.zero()

    fun intake(): Command = startEnd(
        { io.setSpeed(-0.25) },
        { io.setSpeed(0.0) }
    ).until { coralInIntakeBack }

    fun outtake(): Command = startEnd(
        { io.setCurrent(Amps.of(37.0)) },
        { io.setSpeed(0.0) }
    ).until { !(coralInIntakeBack || coralInIntakeFront) }
}
