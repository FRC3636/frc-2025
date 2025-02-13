package com.frcteam3636.frc2025.subsystems.manipulator

import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.utils.math.degreesPerSecond
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
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

    private fun waitForIntake(): Command = Commands.sequence(
        Commands.waitUntil { inputs.current > Amps.of(0.85) },
        Commands.defer({
            val targetRotations = inputs.position + Rotations.of(1.5)
            Commands.waitUntil { inputs.position > targetRotations }
        }, emptySet())
    )

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


    fun idle(): Command = startEnd({
        io.setSpeed(-0.02)
    }, {
        io.setSpeed(0.0)
    })

    fun intake(): Command = startEnd(
        { io.setSpeed(0.065) },
        { io.setSpeed(0.0) }
    )
        .raceWith(waitForIntake())
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)


    fun outtake(): Command = startEnd(
        { io.setCurrent(Amps.of(37.0)) },
        { io.setSpeed(0.0) }
    )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
}
