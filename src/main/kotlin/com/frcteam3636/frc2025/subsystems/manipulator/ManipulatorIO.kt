package com.frcteam3636.frc2025.subsystems.manipulator


import au.grapplerobotics.CanBridge
import au.grapplerobotics.ConfigurationFailedException
import au.grapplerobotics.LaserCan
import au.grapplerobotics.interfaces.LaserCanInterface
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.inAmps
import com.frcteam3636.frc2025.utils.math.inVolts
import com.frcteam3636.frc2025.utils.math.meters
import com.frcteam3636.frc2025.utils.math.millimeters
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team9432.annotation.Logged

@Logged
open class ManipulatorInputs {
    //    var position = 0.rotations
//    var velocity = 0.rotationsPerSecond
//    var current = 0.amps
    var laserCanDistance = Double.POSITIVE_INFINITY.meters
}

interface ManipulatorIO {
    fun setSpeed(percent: Double)
    fun setCurrent(current: Current)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: ManipulatorInputs)
}

class ManipulatorIOReal : ManipulatorIO {
    private var manipulatorMotor = TalonFX(CTREDeviceId.ManipulatorMotor).apply {
        configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                    Inverted = InvertedValue.CounterClockwise_Positive
                }
            }
        )
    }

    init {
        manipulatorMotor.optimizeBusUtilization()
    }

    private var laserCan = LaserCan(REVMotorControllerId.ManipulatorLaserCAN.num).apply {
        try {
            setRangingMode(LaserCanInterface.RangingMode.SHORT)
            setRegionOfInterest(LaserCanInterface.RegionOfInterest(2, 8, 4, 8))
            setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS)
        } catch (e: ConfigurationFailedException) {
            println(e)
        }
    }

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        manipulatorMotor.set(percent)
    }

    private val currentControl = TorqueCurrentFOC(0.0)

    override fun setCurrent(current: Current) {
        assert(current.inAmps() in -60.0..60.0)
        manipulatorMotor.setControl(currentControl.withOutput(current))
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage.inVolts() in -12.0..12.0)
        manipulatorMotor.setVoltage(voltage.inVolts())
    }

    override fun updateInputs(inputs: ManipulatorInputs) {
//        inputs.velocity = manipulatorMotor.velocity.value
//        inputs.current = manipulatorMotor.supplyCurrent.value
//        inputs.position = manipulatorMotor.position.value

        val measurement = laserCan.measurement
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            inputs.laserCanDistance = measurement.distance_mm.millimeters
        } else {
            inputs.laserCanDistance = Double.POSITIVE_INFINITY.meters
        }
    }

}

class ManipulatorIOSim : ManipulatorIO {
    private var motor = DCMotor.getKrakenX60Foc(1)
    private var system = LinearSystemId.createFlywheelSystem(motor, 1.0, 1.0)
    private var simMotor = FlywheelSim(system, motor, 0.0)

    override fun setSpeed(percent: Double) {
        simMotor.inputVoltage = percent * 12
    }

    override fun setCurrent(current: Current) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: ManipulatorInputs) {
        simMotor.update(Robot.period)
//        inputs.velocity = simMotor.angularVelocity
        simMotor.setAngularVelocity(simMotor.angularVelocityRadPerSec * 0.95)
//        inputs.current = simMotor.currentDrawAmps.amps
    }

}
