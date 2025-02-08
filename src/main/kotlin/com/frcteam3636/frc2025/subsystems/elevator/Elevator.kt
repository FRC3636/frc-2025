package com.frcteam3636.frc2025.subsystems.elevator

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.elevator.ElevatorIOReal.Constants.SPOOL_RADIUS
import com.frcteam3636.frc2025.utils.math.meters
import com.frcteam3636.frc2025.utils.math.radians
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger

object Elevator : Subsystem {
    private val io: ElevatorIO = when (Robot.model) {
        Robot.Model.SIMULATION -> ElevatorIOSim()
        Robot.Model.COMPETITION -> ElevatorIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedElevatorInputs()

    val isPressed get() = inputs.leftCurrent > Amps.of(1.9) || inputs.rightCurrent > Amps.of(1.9)

    var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            Volts.per(Second).of(0.5),
            Volts.of(2.0),
            null,
            {
                SignalLogger.writeString("state", it.toString())
            }
        ),
        SysIdRoutine.Mechanism(
            io::setVoltage,
            null,
            this,
        )
    )

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

    fun runHoming(): Command =
        runEnd({
            io.setVoltage(Volts.of(-1.0))
        }, {
            if (isPressed) {
                io.setEncoderPosition(Meters.of(0.0))
            }
            io.setVoltage(Volts.of(0.0))
        }).until {
            isPressed
        }


    fun sysIdQuasistatic(direction: SysIdRoutine.Direction) =
        sysID.quasistatic(direction)!!

    fun sysIdDynamic(direction: SysIdRoutine.Direction) =
        sysID.dynamic(direction)!!

    enum class Position(val height: Distance) {
        Stowed(Meters.of(0.0)),
        LowBar(Meters.of(Rotations.of(0.79).radians * SPOOL_RADIUS.meters)),
        MidBar(Meters.of(Rotations.of(2.18).radians * SPOOL_RADIUS.meters)),
        HighBar(Meters.of(Rotations.of(4.5).radians * SPOOL_RADIUS.meters)),
//        LowAlgae(Meters.of(0.0)),
//        HighAlgae(Meters.of(0.0)),
    }
}
