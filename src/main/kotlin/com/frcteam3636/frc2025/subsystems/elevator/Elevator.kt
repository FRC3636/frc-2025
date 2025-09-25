package com.frcteam3636.frc2025.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.elevator.ElevatorIOReal.Constants.SPOOL_RADIUS
import com.frcteam3636.frc2025.utils.math.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

object Elevator : Subsystem {
    private val io: ElevatorIO = when (Robot.model) {
        Robot.Model.SIMULATION -> ElevatorIOSim()
        Robot.Model.COMPETITION -> ElevatorIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedElevatorInputs()

    val isPressed get() = inputs.leftCurrent > 1.9.amps || inputs.rightCurrent > 1.9.amps
    private var desiredHeight = 0.meters
    val isAtTarget get() = (inputs.height - desiredHeight) < 0.5.inches

    var position = Position.Stowed

    var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            0.5.voltsPerSecond,
            2.volts,
            null
        ) {
            SignalLogger.writeString("state", it.toString())
        },
        SysIdRoutine.Mechanism(
            io::setVoltage,
            null,
            this,
        )
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Elevator", inputs)
        // idea stolen from 254's 2025 elevator code
        if (Robot.isDisabled && inputs.height < 0.0.meters)
            io.setEncoderPosition(0.0.meters)
    }

    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return io.getStatusSignals()
    }

    fun setTargetHeight(position: Position): Command =
        startEnd({
            this.position = position
            io.runToHeight(position.height)
            desiredHeight = position.height
        }, {})
            .until { abs(inputs.height.inMeters() - position.height.inMeters()) < 0.75.inches.inMeters() }

    fun setTargetHeightAlgae(position: Position): Command =
        startEnd({
            this.position = position
            io.runToHeightWithOverride(position.height, 200.0.rotationsPerSecond, 20.0.rotationsPerSecondPerSecond)
            desiredHeight = position.height
        }, {})
            .until { abs(inputs.height.inMeters() - position.height.inMeters()) < 0.75.inches.inMeters() }

    fun runHoming(): Command =
        runEnd({
            io.setVoltage((-1.0).volts)
        }, {
            if (isPressed) {
                io.setEncoderPosition(0.meters)
            }
            io.setVoltage(0.volts)
        }).until {
            isPressed
        }

    /** Doesn't require this subsystem */
    fun coast(): Command = Commands.startEnd({
        io.setBrakeMode(false)
    }, {
        io.setBrakeMode(true)
    })

    @Suppress("unused")
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction) =
        sysID.quasistatic(direction)!!

    @Suppress("unused")
    fun sysIdDynamic(direction: SysIdRoutine.Direction) =
        sysID.dynamic(direction)!!

    enum class Position(val height: Distance) {
        Stowed(0.meters),
        LowBar(0.79.rotations.toLinear(SPOOL_RADIUS)),
        AlgaeMidBar(1.5.rotations.toLinear(SPOOL_RADIUS)),
        MidBar(2.2.rotations.toLinear(SPOOL_RADIUS)),

        HighBar(4.45.rotations.toLinear(SPOOL_RADIUS)),
//        LowAlgae(Meters.of(0.0)),
//        HighAlgae(Meters.of(0.0)),
    }
}
