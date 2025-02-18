package com.frcteam3636.frc2025.subsystems.climb

import com.frcteam3636.frc2025.Robot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Climb : Subsystem {
    private val io: ClimbIO = when (Robot.model) {
        Robot.Model.SIMULATION -> TODO()
        Robot.Model.COMPETITION -> ClimbIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedClimbInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climb", inputs)
    }

    fun up(): Command = startEnd(
        {
            io.setSpeed(0.50)
        },
        {
            io.setSpeed(0.0)
        }
    )

    fun down(): Command = startEnd(
        {
            io.setSpeed(-0.50)
        },
        {
            io.setSpeed(0.0)
        }
    )

}