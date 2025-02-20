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
import edu.wpi.first.math.geometry.Rotation3d
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
    var rotation: Rotation3d

    /**
     * The rotational velocity of the robot on its yaw axis.
     */
    val velocity: AngularVelocity

    /** Whether the gyro is connected. */
    val connected: Boolean

    fun periodic() {}
}

class GyroNavX(private val ahrs: AHRS) : Gyro {

    private var offset = Rotation2d()

    init {
        Logger.recordOutput("NavXGyro/Offset", offset)
    }

    override var rotation: Rotation3d
        get() = ahrs.rotation3d + Rotation3d(offset)
        set(goal) {
            offset = goal.toRotation2d() - ahrs.rotation2d
            Logger.recordOutput("NavXGyro/Offset", offset)
        }

    override val velocity: AngularVelocity
        get() = 0.degreesPerSecond // NavX get rate broken... use the Pigeon lol

    override val connected
        get() = ahrs.isConnected
}

class GyroPigeon(private val pigeon: Pigeon2) : Gyro {
    init {
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, pigeon.yaw, pigeon.pitch, pigeon.roll)
    }

    override var rotation: Rotation3d
        get() = pigeon.rotation3d
        set(goal) {
            pigeon.setYaw(goal.toRotation2d().measure)
        }

    override val velocity: AngularVelocity
        get() = pigeon.angularVelocityZWorld.value

    override val connected
        get() = pigeon.yaw.status.isOK
}

class GyroMapleSim(val gyroSimulation: GyroSimulation) : Gyro {
    override var rotation: Rotation3d
        get() = Rotation3d(gyroSimulation.gyroReading)
        set(value) {
            gyroSimulation.setRotation(value.toRotation2d())
        }
    override val velocity: AngularVelocity
        get() = gyroSimulation.measuredAngularVelocity
    override val connected: Boolean
        get() = true
}
