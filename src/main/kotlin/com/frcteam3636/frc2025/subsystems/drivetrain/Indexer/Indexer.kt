package com.frcteam3636.frc2025.subsystems.drivetrain.Indexer

import com.frcteam3636.frc2025.Robot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

object Indexer: Subsystem {
    private var io: IndexerIO = when (Robot.model) {
        Robot.Model.SIMULATION -> IndexerIOSim()
        Robot.Model.COMPETITION -> IndexerIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedIndexerInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)

        Intake.indexerAngleLigament.angle = inputs.position.`in`(Degrees)
    }

    /**
     * Runs the indexer forward if balloon matches the current alliance.
     * Does not run if no balloon.
     * Reverses if balloon is wrong alliance.
     */
    fun indexCoral(): Command = runEnd(
        {io.setSpinSpeed(0.5)},
        {io.setSpinSpeed(0.0)}
    )

    fun outtakeCoral(): Command = runEnd(
        {io.setSpinSpeed(-0.5)},
        {io.setSpinSpeed(0.0)}
    )
}
