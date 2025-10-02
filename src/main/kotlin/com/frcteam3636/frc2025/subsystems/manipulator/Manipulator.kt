package com.frcteam3636.frc2025.subsystems.manipulator

import com.ctre.phoenix6.BaseStatusSignal
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.alignStatePublisher
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.utils.math.amps
import com.frcteam3636.frc2025.utils.math.volts
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Manipulator : Subsystem {
    private val io: ManipulatorIO = when (Robot.model) {
        Robot.Model.SIMULATION -> ManipulatorIOSim()
        Robot.Model.COMPETITION -> ManipulatorIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedManipulatorInputs()

    var coralState: CoralState = CoralState.NONE
        private set(value) {
            field = value
            rgbPublisher.set(value.ordinal.toLong())
        }

    private var rgbPublisher = NetworkTableInstance.getDefault().getIntegerTopic("RGB/Coral State").publish()

//    private var mechanism = LoggedMechanism2d(100.0, 100.0)
//    private var motorAngleVisualizer =
//        LoggedMechanismLigament2d("Manipulator Motor Angle", 40.0, 0.0, 5.0, Color8Bit(Color.kRed))

    var isIntakeRunning = false

//    init {
//        mechanism.getRoot("Manipulator", 50.0, 50.0).apply {
//            append(motorAngleVisualizer)
//        }
//    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Manipulator", inputs)

//        motorAngleVisualizer.angle += inputs.velocity.inDegreesPerSecond() * Robot.period
//        Logger.recordOutput("/Manipulator/Mechanism", mechanism)
        Logger.recordOutput("Manipulator/Is Intake Running", isIntakeRunning)
        Logger.recordOutput("Manipulator/Coral State", coralState.name)
    }


    fun idle(): Command = startEnd({
        io.setSpeed(-0.02)
    }, {
        io.setSpeed(0.0)
    })

    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return io.getStatusSignals()
    }

    fun intake(driverFeedback: Command = Commands.none()): Command = Commands.sequence(
        runOnce { io.setVoltage(2.0.volts) },
        Commands.waitUntil { inputs.isCoralDetected },
        runOnce { io.setVoltage(1.volts) },
        Commands.runOnce({
            coralState = CoralState.TRANSIT
        }),
        Commands.waitUntil { !inputs.isCoralDetected },
        runOnce { io.setSpeed(-0.02) },
        Commands.runOnce({
            coralState = CoralState.HELD
            driverFeedback.schedule()
        }),
    )
        .onlyWhile {
            isIntakeRunning
        }
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)

    fun intakeAuto(): Command = Commands.sequence(
        runOnce { io.setVoltage(2.0.volts) },
        Commands.waitUntil { inputs.isCoralDetected },
        runOnce { io.setVoltage(1.volts) },
        Commands.runOnce({
            coralState = CoralState.TRANSIT
        }),
        Commands.waitUntil { !inputs.isCoralDetected },
        runOnce { io.setSpeed(-0.02) },
        Commands.runOnce({
            coralState = CoralState.HELD
        }),
    )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)

    fun outtake(): Command = startEnd(
        {
            if (Elevator.position != Elevator.Position.HighBar) {
                io.setCurrent(40.amps)
            } else {
                io.setCurrent(50.amps)
            }
        },
        {
            io.setSpeed(0.0)
            coralState = CoralState.NONE
            alignStatePublisher.set(Drivetrain.AlignState.NotRunning.raw)
        }
    )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)

    fun outtakeAlgae(): Command = startEnd(
        { io.setCurrent((-60).amps) },
        {
            io.setSpeed(0.0)
            coralState = CoralState.NONE
            alignStatePublisher.set(Drivetrain.AlignState.NotRunning.raw)
        }
    )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)

    fun intakeAlgae(): Command = startEnd(
        { io.setVoltage(3.0.volts) },
        {
            io.setSpeed(0.0)
        }
    ).onlyWhile {
        isIntakeRunning
    }.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
}

enum class CoralState {
    NONE,
    HELD,
    TRANSIT
}
