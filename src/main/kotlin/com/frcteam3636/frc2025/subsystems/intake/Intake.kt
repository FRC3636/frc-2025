package com.frcteam3636.frc2025.subsystems.intake

import com.frcteam3636.frc2025.Robot
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Intake: Subsystem {

    private val io: IntakeIO = when (Robot.model) {
        Robot.Model.SIMULATION -> IntakeIOSim()
        Robot.Model.COMPETITION -> IntakeIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedIntakeInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }

    fun downAndIntake(): Command =
        startEnd(
            {
                io.setSpeed(0.5)
                io.pivotToAngle(Radians.of(1.0))
            },
            {
                io.setSpeed(0.0)
                io.pivotToAngle(Radians.of(-1.0))
            }
        )

    fun downAndOuttake(): Command =
        startEnd(
            {
                io.setSpeed(-0.5)
                io.pivotToAngle(Radians.of(1.0))
            },
            {
                io.setSpeed(0.0)
                io.pivotToAngle(Radians.of(-1.0))
            }
        )
}