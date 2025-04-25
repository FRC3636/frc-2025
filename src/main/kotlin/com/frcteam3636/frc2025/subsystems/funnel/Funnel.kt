package com.frcteam3636.frc2025.subsystems.funnel

import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.utils.math.volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger


object Funnel : Subsystem {
    private val io: FunnelIO = when (Robot.model) {
        Robot.Model.SIMULATION -> FunnelIOSim()
        Robot.Model.COMPETITION -> FunnelIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedFunnelInputs()

//    private var mechanism = LoggedMechanism2d(100.0, 100.0)
//    private var motorAngleVisualiser =
//        LoggedMechanismLigament2d("Funnel Motor Angle", 40.0, 0.0, 5.0, Color8Bit(Color.kRed))

    init {
//        mechanism.getRoot("Funnel", 50.0, 50.0).apply { append(motorAngleVisualiser) }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Funnel", inputs)

//        motorAngleVisualiser.angle += inputs.rollerVelocity.inDegreesPerSecond() * Robot.period
//        Logger.recordOutput("/Funnel/mechanism", mechanism)
    }

    fun intake(): Command = startEnd(
        {
            io.setVoltage(10.0.volts)
        },
        {
            io.setSpeed(0.0)
        }
    )

    fun outtake(): Command = startEnd(
        {
            io.setSpeed(-0.25)
        },
        {
            io.setSpeed(0.0)
        }
    )

}
