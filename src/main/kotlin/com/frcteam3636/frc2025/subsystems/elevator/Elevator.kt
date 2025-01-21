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
        Stowed(Meters.of(0.0)), // Not true but will change with the design, make an actual constant after a real robot is built
        Trough(Meters.of(0.418)),
        LowBar(Meters.of(0.765124)),
        MidBar(Meters.of(1.161953)),
        HighBar(Meters.of(1.809750)),
//        LowAlgae(Meters.of(0.0)), // No algae for now according to the cad people
//        HighAlgae(Meters.of(0.0)),
    }
}