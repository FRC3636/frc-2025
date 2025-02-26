package com.frcteam3636.frc2025.subsystems.manipulator

import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.utils.LimelightHelpers
import com.frcteam3636.frc2025.utils.math.*
import edu.wpi.first.networktables.NetworkTableInstance
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

    private var coralState: CoralState = CoralState.NONE
        set(value) {
            field = value
            rgbPublisher.set(value.ordinal.toLong())
        }
    private var rgbPublisher = NetworkTableInstance.getDefault().getIntegerTopic("RGB/Coral State").publish()

    private var mechanism = LoggedMechanism2d(100.0, 100.0)
    private var motorAngleVisualizer =
        LoggedMechanismLigament2d("Manipulator Motor Angle", 40.0, 0.0, 5.0, Color8Bit(Color.kRed))

    private fun waitForIntake(): Command = Commands.sequence(
        Commands.waitUntil { inputs.laserCanDistance < 0.2.meters },
        Commands.waitUntil { inputs.laserCanDistance > 0.2.meters },
        Commands.runOnce({
            coralState = CoralState.HELD
            blinkLimelight().schedule()
        }),
    )

    var isIntakeRunning = false

    init {
        mechanism.getRoot("Manipulator", 50.0, 50.0).apply {
            append(motorAngleVisualizer)
        }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Manipulator", inputs)

        motorAngleVisualizer.angle += inputs.velocity.inDegreesPerSecond() * Robot.period
        Logger.recordOutput("/Manipulator/Mechanism", mechanism)
    }

    private fun blinkLimelight(): Command = Commands.runOnce({
        LimelightHelpers.setLEDMode_ForceBlink("limelight-rear")
    })
        .andThen(Commands.waitSeconds(0.3))
        .finallyDo { ->
            LimelightHelpers.setLEDMode_PipelineControl("limelight-rear")
        }


    fun idle(): Command = startEnd({
        io.setSpeed(-0.02)
    }, {
        io.setSpeed(0.0)
    })

    fun intake(): Command = startEnd(
        { io.setVoltage(0.5.volts) },
        { io.setSpeed(0.0) }
    )
        .raceWith(waitForIntake())
        .onlyWhile {
            isIntakeRunning
        }
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)

    fun intakeNoRaceWithOutInterrupt(): Command = run(
        { io.setVoltage(0.78.volts) },
    )

    fun outtake(): Command = startEnd(
        { io.setCurrent(60.amps) },
        {
            io.setSpeed(0.0)
            coralState = CoralState.NONE
        }
    )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)

    fun outtakeAlgae(): Command = startEnd(
        { io.setCurrent(-60.amps) },
        {
            io.setSpeed(0.0)
            coralState = CoralState.NONE
        }
    )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
}

enum class CoralState {
    NONE,
    HELD
}
