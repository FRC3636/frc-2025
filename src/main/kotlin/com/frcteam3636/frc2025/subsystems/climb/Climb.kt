package com.frcteam3636.frc2025.subsystems.climb

import com.frcteam3636.frc2025.Robot
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import kotlin.time.Duration.Companion.seconds

object Climb: Subsystem {
    private val io: ClimbIO = when (Robot.model) {
        Robot.Model.SIMULATION -> ClimbIOSim()
        Robot.Model.COMPETITION -> ClimbIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedClimbInputs()

    private var mechanism = LoggedMechanism2d(100.0, 100.0)
    private var motorAngleVisualizer =
        LoggedMechanismLigament2d("Climber motor angle", 40.0, 0.0, 5.0, Color8Bit(Color.kRed))

    init {
        mechanism.getRoot("Climber", 50.0, 50.0).apply {
            append(motorAngleVisualizer)
        }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climb", inputs)
        Logger.recordOutput("/Climber/Mechanism", mechanism)
    }

    fun moveToPosition(position: Position) =
        startEnd({
            io.setSpeed(0.5)
        }, {
            io.setSpeed(-1.0)
        })!!

    enum class Position(val angle: Angle) {
        Climb(Radians.of(1.57079632679)),
        Stowed(Radians.of(3.14159265359)),
        Deployed(Radians.of(0.0))
    }
}