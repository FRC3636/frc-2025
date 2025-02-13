package com.frcteam3636.frc2025.subsystems.climb

import com.frcteam3636.frc2025.Robot
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Climb: Subsystem {
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

    fun moveToPosition(position: Position) =
        startEnd({
            io.turnToAngle(position.angle)
        }, {
            io.turnToAngle(inputs.position)
        })!!

    enum class Position(val angle: Angle) {
        Climb(Radians.of(1.57079632679)),
        Stowed(Radians.of(3.14159265359)),
        Deployed(Radians.of(0.0))
    }
}