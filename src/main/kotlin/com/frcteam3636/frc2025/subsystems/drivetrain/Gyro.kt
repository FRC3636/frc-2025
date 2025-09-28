package com.frcteam3636.frc2025.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.hardware.Pigeon2
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.utils.math.degreesPerSecond
import com.frcteam3636.frc2025.utils.math.radiansPerSecond
import com.frcteam3636.frc2025.utils.swerve.PerCorner
import com.frcteam3636.frc2025.utils.swerve.translation2dPerSecond
import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.AngularVelocity
import org.ironmaple.simulation.drivesims.GyroSimulation
import org.littletonrobotics.junction.Logger
import kotlin.math.sign

interface Gyro {
    /**
     * The current rotation of the robot.
     * This can be set to a different value to change the gyro's offset.
     */
    var rotation: Rotation2d

    /**
     * The rotational velocity of the robot on its yaw axis.
     */
    val velocity: AngularVelocity

    /** Whether the gyro is connected. */
    val connected: Boolean

    fun getStatusSignals(): Array<BaseStatusSignal> {
        return arrayOf()
    }

    fun periodic() {}
}

class GyroNavX(private val ahrs: AHRS) : Gyro {

    private var offset = Rotation2d()

    init {
        Logger.recordOutput("NavXGyro/Offset", offset)
    }

    override var rotation: Rotation2d
        get() = offset + ahrs.rotation2d
        set(goal) {
            offset = goal - ahrs.rotation2d
            Logger.recordOutput("NavXGyro/Offset", offset)
        }

    override val velocity: AngularVelocity
        get() = 0.degreesPerSecond // NavX get rate broken... use the Pigeon lol

    override val connected
        get() = ahrs.isConnected
}

class GyroPigeon(private val pigeon: Pigeon2) : Gyro {
    init {
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, pigeon.yaw, pigeon.pitch, pigeon.roll, pigeon.angularVelocityZWorld)
    }

    override var rotation: Rotation2d
        // Basically just pigeon.rotation2d but bypasses the refresh
        get() = Rotation2d.fromDegrees(pigeon.getYaw(false).valueAsDouble)
        set(goal) {
            pigeon.setYaw(goal.measure)
        }

    override val velocity: AngularVelocity
        get() = pigeon.getAngularVelocityZWorld(false).value

    override val connected
        get() = pigeon.getYaw(false).status.isOK

    override fun getStatusSignals(): Array<BaseStatusSignal> {
        return arrayOf(pigeon.getYaw(false), pigeon.getPitch(false), pigeon.getRoll(false), pigeon.getAngularVelocityZWorld(false))
    }
}

class GyroSim(private val modules: PerCorner<SwerveModule>) : Gyro {
    override var rotation = Rotation2d()
    override var velocity: AngularVelocity = 0.radiansPerSecond
    override val connected = true

    override fun periodic() {
        // Calculate the average translation velocity of each module
        val moduleVelocities = modules.map { it.state.translation2dPerSecond }
        val translationVelocity = moduleVelocities.reduce(Translation2d::plus) / moduleVelocities.size.toDouble()

        // Use the front left module's rotational velocity to calculate the yaw velocity
        val rotationalVelocities = moduleVelocities.map { it - translationVelocity }
        val yawVelocity =
            sign(rotationalVelocities.frontLeft.y) * rotationalVelocities.frontLeft.norm /
                    Drivetrain.Constants.MODULE_POSITIONS.frontLeft.translation.norm

        velocity = yawVelocity.radiansPerSecond
        rotation += Rotation2d(yawVelocity) * Robot.period
    }
}

class GyroMapleSim(val gyroSimulation: GyroSimulation) : Gyro {
    override var rotation: Rotation2d
        get() = gyroSimulation.gyroReading
        set(value) {
            gyroSimulation.setRotation(value)
        }
    override val velocity: AngularVelocity
        get() = gyroSimulation.measuredAngularVelocity
    override val connected: Boolean
        get() = true
}
