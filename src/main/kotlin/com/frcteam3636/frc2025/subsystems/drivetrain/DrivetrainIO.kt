package com.frcteam3636.frc2025.subsystems.drivetrain

import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.Pigeon2
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.BUMPER_LENGTH
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.BUMPER_WIDTH
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.TRACK_WIDTH
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.WHEEL_BASE
import com.frcteam3636.frc2025.utils.math.TAU
import com.frcteam3636.frc2025.utils.swerve.PerCorner
import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.COTS.WHEELS
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import org.littletonrobotics.junction.Logger
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


    open fun updateInputs(inputs: DrivetrainInputs) {
        gyro.periodic()
        modules.forEach(SwerveModule::periodic)

        inputs.gyroRotation = gyro.rotation
        inputs.gyroVelocity = gyro.velocity
        inputs.gyroConnected = gyro.connected
        inputs.measuredStates = modules.map { it.state }
        inputs.measuredPositions = modules.map { it.position }
    }

    var desiredStates: PerCorner<SwerveModuleState>
        get() = modules.map { it.desiredState }
        set(value) {
            modules.zip(value).forEach { (module, state) -> module.desiredState = state }
        }
}

/** Drivetrain I/O layer that uses real swerve modules along with a NavX gyro. */
class DrivetrainIOReal(override val modules: PerCorner<SwerveModule>) : DrivetrainIO() {
    override val gyro = when (Robot.model) {
        Robot.Model.SIMULATION -> GyroSim(modules)
        Robot.Model.COMPETITION -> GyroPigeon(Pigeon2(CTREDeviceId.PigeonGyro))
        Robot.Model.PROTOTYPE -> GyroNavX(AHRS(AHRS.NavXComType.kMXP_SPI))
    }

    companion object {
        fun fromKrakenSwerve() =
            DrivetrainIOReal(
                MODULE_POSITIONS.zip(Drivetrain.Constants.KRAKEN_MODULE_CAN_IDS)
                    .map { (position, ids) ->
                        val (driveId, turnId) = ids
                        MAXSwerveModule(
                            DrivingTalon(driveId),
                            turnId,
                            position.rotation
                        )
                    })

        fun fromNeoSwerve() =
            DrivetrainIOReal(
                MODULE_POSITIONS.zip(Drivetrain.Constants.MODULE_CAN_IDS_PRACTICE)
                    .map { (position, ids) ->
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
    // Create and configure a drivetrain simulation configuration
    val driveTrainSimulationConfig: DriveTrainSimulationConfig =
        DriveTrainSimulationConfig.Default() // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofPigeon2()) // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(
                // FIXME: Calculate values
                SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),  // Drive motor is a Kraken X60
                    DCMotor.getNeo550(1),  // Steer motor is a Neo 550
                    (45.0 * 22.0) / (14.0 * 15.0),
                    9424.0 / 203.0,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    WHEEL_RADIUS,
                    KilogramSquareMeters.of(0.02),
                    WHEELS.SLS_PRINTED_WHEELS.cof
                )
            )
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(
                WHEEL_BASE,
                TRACK_WIDTH
            ) // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(BUMPER_WIDTH, BUMPER_LENGTH)

            .withCustomModuleTranslations(
                MODULE_POSITIONS.map { it.translation }.toTypedArray()
            )

    // Create a swerve drive simulation
    val swerveDriveSimulation = SwerveDriveSimulation(
        // Specify Configuration
        driveTrainSimulationConfig,
        // Specify starting pose
        Pose2d(3.0, 3.0, Rotation2d())
    )

    override val modules = PerCorner.generate { SimSwerveModule(swerveDriveSimulation.modules[it.ordinal]) }
    override val gyro = GyroMapleSim(swerveDriveSimulation.gyroSimulation)

    init {
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation)
    }

    override fun updateInputs(inputs: DrivetrainInputs) {
        super.updateInputs(inputs)
        Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation.simulatedDriveTrainPose)
    }

    // Register the drivetrain simulation to the default simulation world
}
