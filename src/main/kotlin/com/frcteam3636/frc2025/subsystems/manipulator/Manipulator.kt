package com.frcteam3636.frc2025.subsystems.manipulator

import com.frcteam3636.frc2025.Robot
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger

object Manipulator: Subsystem {
    private val io: ManipulatorIO = when (Robot.model){
        Robot.Model.SIMULATION -> ManipulatorIOSim()
        Robot.Model.COMPETITION -> ManipulatorIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedManipulatorInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Manipulator", inputs)
    }

    val coralInIntakeBack = Trigger{inputs.backUltrasonicDistance < Meters.zero()}
    val coralInIntakeFront = Trigger{inputs.frontUltrasonicDistance < Meters.zero()}

    fun intake(): Command = startEnd(
        { io.setSpeed(0.5) },
        { io.setSpeed(0.0) }
    )
        .raceWith(
            Commands.waitUntil(coralInIntakeBack)
        )

    fun outtake(): Command = startEnd(
        { io.setSpeed(-0.5) },
        { io.setSpeed(0.0) }
    )
        .raceWith(
            Commands.waitUntil(coralInIntakeBack.negate().and(coralInIntakeFront.negate()))
        )

}