package com.frcteam3636.frc2025.subsystems.elevator

import com.frcteam3636.frc2025.Robot
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Elevator: Subsystem {
    private val io: ElevatorIO = when (Robot.model) {
        Robot.Model.SIMULATION -> TODO()
        Robot.Model.COMPETITION -> ElevatorIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedElevatorInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Elevator", inputs)
    }

    fun setTargetHeight(position: Position): Command =
        startEnd({
        io.runToHeight(position.height)
    }, {
        io.runToHeight(inputs.height)
    })!!

//    fun sysIdQuasistatic(direction: Direction) =
//        sysID.quasistatic(direction)!!
//
//    fun sysIdDynamic(direction: Direction) =
//        sysID.dynamic(direction)!!

    enum class Position(val height: Distance) {
        Stowed(Meters.of(0.0)),
        Trough(Meters.of(0.0)),
        LowBar(Meters.of(0.0)),
        MidBar(Meters.of(0.0)),
        HighBar(Meters.of(0.0)),
        LowAlgae(Meters.of(0.0)),
        HighAlgae(Meters.of(0.0)),
    }
}