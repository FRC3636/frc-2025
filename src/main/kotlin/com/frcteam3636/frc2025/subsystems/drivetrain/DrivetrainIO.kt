package com.frcteam3636.frc2025.subsystems.drivetrain

import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.Pigeon2
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.utils.swerve.PerCorner
import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.DegreesPerSecond
import org.team9432.annotation.Logged

@Logged
open class DrivetrainInputs {
    var gyroRotation = Rotation2d()
    var gyroVelocity = DegreesPerSecond.zero()!!
    var gyroConnected = true
    var measuredStates = PerCorner.generate { SwerveModuleState() }
    var measuredPositions = PerCorner.generate { SwerveModulePosition() }
}


abstract class DrivetrainIO {
    protected abstract val gyro: Gyro
    abstract val modules: PerCorner<out SwerveModule>


    fun updateInputs(inputs: DrivetrainInputs) {
        gyro.periodic()
        modules.forEach(SwerveModule::periodic)

        inputs.gyroRotation = gyro.rotation
        inputs.gyroVelocity = gyro.velocity
        inputs.gyroConnected = gyro.connected
        inputs.measuredStates = modules.map { it.state }
        inputs.measuredPositions = modules.map { it.position }
    }

    fun setGyro(rotation: Rotation2d) {
        gyro.rotation = rotation
    }

    var desiredStates: PerCorner<SwerveModuleState>
        get() = modules.map { it.desiredState }
        set(value) {
            modules.zip(value).forEach { (module, state) -> module.desiredState = state }
        }
}

/** Drivetrain I/O layer that uses real swerve modules along with a NavX gyro. */
class DrivetrainIOReal(override val modules: PerCorner<SwerveModule>) : DrivetrainIO() {
    override val gyro = when(Robot.model) {
        Robot.Model.SIMULATION -> GyroSim(modules)
        Robot.Model.COMPETITION -> GyroPigeon(Pigeon2(CTREDeviceId.PigeonGyro))
        Robot.Model.PROTOTYPE -> GyroNavX(AHRS(AHRS.NavXComType.kMXP_SPI))
    }

    companion object {
        fun fromKrakenSwerve() =
            DrivetrainIOReal(
                Drivetrain.Constants.MODULE_POSITIONS.zip(Drivetrain.Constants.KRAKEN_MODULE_CAN_IDS)
                    .map { (position, ids) ->
                        val (driveId, turnId) = ids
                        MAXSwerveModule(
                            DrivingTalon(driveId),
                            turnId,
                            position.rotation
                        )
                    })

        fun fromNeoSwerve() =
            DrivetrainIOReal(Drivetrain.Constants.MODULE_POSITIONS.zip(Drivetrain.Constants.MODULE_CAN_IDS_PRACTICE).map { (position, ids) ->
                val (driveId, turnId) = ids
                MAXSwerveModule(
                    DrivingSparkMAX(driveId),
                    turnId,
                    position.rotation
                )
            })
    }
}

///** Drivetrain I/O layer that uses simulated swerve modules along with a simulated gyro with an angle based off their movement. */
//class DrivetrainIOSim : DrivetrainIO() {
//    override val modules = PerCorner.generate { SimSwerveModule() }
//    override val gyro = GyroSim(modules.map { it })
//}
