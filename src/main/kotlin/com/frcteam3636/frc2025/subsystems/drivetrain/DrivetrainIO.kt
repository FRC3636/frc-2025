package com.frcteam3636.frc2025.subsystems.drivetrain

import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.Pigeon2
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.BUMPER_LENGTH
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.BUMPER_WIDTH
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.TRACK_WIDTH
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.WHEEL_BASE
import com.frcteam3636.frc2025.utils.swerve.PerCorner
import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Inches
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.GyroSimulation
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.team9432.annotation.Logged


@Logged
open class DrivetrainInputs {
    var gyroRotation = Rotation3d()
    var measuredStates = PerCorner.generate { SwerveModuleState() }
    var measuredPositions = PerCorner.generate { SwerveModulePosition() }
    var gyroConnected = true
}


abstract class DrivetrainIO {
    protected abstract val gyro: Gyro
    abstract val modules: PerCorner<out SwerveModule>


    fun updateInputs(inputs: DrivetrainInputs) {
        gyro.periodic()
        modules.forEach(SwerveModule::periodic)

        inputs.gyroRotation = gyro.rotation
        inputs.measuredStates = modules.map { it.state }
        inputs.measuredPositions = modules.map { it.position }
        inputs.gyroConnected = gyro.connected
    }

    fun setGyro(rotation: Rotation3d) {
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

/** Drivetrain I/O layer that uses simulated swerve modules along with a simulated gyro with an angle based off their movement. */
class DrivetrainIOSim : DrivetrainIO() {
    override val modules = PerCorner.generate { SimSwerveModule() }
    override val gyro = GyroMapleSim(GyroSimulation(.001, .05))

    // Create and configure a drivetrain simulation configuration
    val driveTrainSimulationConfig: DriveTrainSimulationConfig =
        DriveTrainSimulationConfig.Default() // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofPigeon2()) // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(
                COTS.ofMark4(
                    DCMotor.getKrakenX60(1),  // Drive motor is a Kraken X60
                    DCMotor.getNeo550(1),  // Steer motor is a Neo 550
                    COTS.WHEELS.COLSONS.cof,  // Use the COF for Colson Wheels
                    3
                )
            ) // L3 Gear ratio
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(
                WHEEL_BASE,
                TRACK_WIDTH
            ) // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(BUMPER_WIDTH, BUMPER_LENGTH)

    // Create a swerve drive simulation
    val swerveDriveSimulation = SwerveDriveSimulation(
        // Specify Configuration
        driveTrainSimulationConfig,
        // Specify starting pose
        Pose2d(3.0, 3.0, Rotation2d())
    )

    init {
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation)
    }

    // Register the drivetrain simulation to the default simulation world
}
