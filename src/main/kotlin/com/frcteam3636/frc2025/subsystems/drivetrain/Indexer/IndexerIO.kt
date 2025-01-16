package com.frcteam3636.frc2025.subsystems.drivetrain.Indexer

import com.frcteam3636.frc2025.SparkFlex
import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.utils.math.TAU
import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team9432.annotation.Logged

@Logged
open class IndexerInputs {
    var indexerVelocity = RotationsPerSecond.zero()!!
    var indexerCurrent = Amps.zero()!!
    var position = Radians.zero()!!
}

interface IndexerIO {
    fun updateInputs(inputs: IndexerInputs)

    fun setSpinSpeed(speed: Double)
    // percent of full speed
}

class IndexerIOReal : IndexerIO {
    private var indexerMotor =
        SparkFlex(
            REVMotorControllerId.IndexerMotor,
            SparkLowLevel.MotorType.kBrushless
        )

    override fun updateInputs(inputs: IndexerInputs) {
        inputs.indexerVelocity = Rotations.per(Minute).of(indexerMotor.encoder.velocity)
        inputs.indexerCurrent = Amps.of(indexerMotor.outputCurrent)
        inputs.position = Rotations.of(indexerMotor.encoder.position)
    }

    override fun setSpinSpeed(speed: Double) {
        assert(speed in -1.0..1.0)
        println("Speed: $speed")
        indexerMotor.set(speed)
    }
}

class IndexerIOSim: IndexerIO {
    val flywheelSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNeoVortex(1),
            0.00942358695924,
            1.6
            ),
        DCMotor.getNeoVortex(1)
    )

    override fun updateInputs(inputs: IndexerInputs) {
        flywheelSim.update(Robot.period)
        inputs.indexerVelocity = RadiansPerSecond.of(flywheelSim.angularVelocityRadPerSec)
        inputs.position += Radians.of(flywheelSim.angularVelocityRadPerSec * Robot.period)
        inputs.position = Radians.of(inputs.position.`in`(Radians).mod(TAU))
    }

    override fun setSpinSpeed(speed: Double) {
        assert(speed in -1.0..1.0)
        flywheelSim.setInputVoltage(speed*12.0)
    }
}

class IndexerIOPrototype: IndexerIO {
    override fun updateInputs(inputs: IndexerInputs) {
    }

    override fun setSpinSpeed(speed: Double) {
    }
}