package com.frcteam3636.frc2025.subsystems.drivetrain

import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.BRAKE_POSITION
//import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain.Constants.DEFAULT_PATHING_CONSTRAINTS
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.FREE_SPEED
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.JOYSTICK_DEADBAND
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.ROTATION_PID_GAINS
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.ROTATION_SENSITIVITY
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.TRANSLATION_SENSITIVITY
import com.frcteam3636.frc2025.utils.ElasticWidgets
import com.frcteam3636.frc2025.utils.math.*
import com.frcteam3636.frc2025.utils.swerve.PerCorner
import com.frcteam3636.frc2025.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.frc2025.utils.swerve.toCornerSwerveModuleStates
import com.frcteam3636.frc2025.utils.translation2d
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.controllers.PPLTVController
import com.pathplanner.lib.pathfinding.Pathfinding
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig
//import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.TimeUnit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.littletonrobotics.junction.Logger
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.abs

/** A singleton object representing the drivetrain. */
object Drivetrain : Subsystem, Sendable {
    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> TODO()
        Robot.Model.COMPETITION -> DrivetrainIOReal.fromKrakenSwerve()
        Robot.Model.PROTOTYPE -> DrivetrainIOReal.fromNeoSwerve()
    }
    val inputs = LoggedDrivetrainInputs()

    private val absolutePoseIOs = mapOf(
        "Limelight" to LimelightPoseIOReal(
            "limelight",
        )
    ).mapValues { Pair(it.value, AbsolutePoseIO.Inputs()) }

    /** Helper for converting a desired drivetrain velocity into the speeds and angles for each swerve module */
    private val kinematics =
        SwerveDriveKinematics(
            *Constants.MODULE_POSITIONS.map(Pose2d::getTranslation).toTypedArray()
        )

    /** Helper for estimating the location of the drivetrain on the field */
    private val poseEstimator =
        SwerveDrivePoseEstimator(
            kinematics, // swerve drive kinematics
            inputs.gyroRotation.toRotation2d(), // initial gyro rotation
            inputs.measuredPositions.toTypedArray(), // initial module positions
            Pose2d(), // initial pose
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5.0)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10.0))
        )

    /** Whether every camera used for pose estimation is connected. */
    val allCamerasConnected
        get() = absolutePoseIOs.values.all { it.first.cameraConnected }

    var gyroRate = RadiansPerSecond.zero()!!
        private set

    init {
        Pathfinding.setPathfinder(
            LocalADStarAK()
        )

        // FIXME: Update for 2025
        AutoBuilder.configure(
            this::estimatedPose,
            this::estimatedPose::set,
            this::measuredChassisSpeeds,
            this::desiredChassisSpeeds::set,
            PPHolonomicDriveController(
                when(Robot.model) {
                    Robot.Model.SIMULATION -> DRIVING_PID_GAINS_TALON
                    Robot.Model.COMPETITION -> DRIVING_PID_GAINS_TALON
                    Robot.Model.PROTOTYPE -> DRIVING_PID_GAINS_NEO
                }.toPPLib(),
                ROTATION_PID_GAINS.toPPLib()
            ),
            Constants.PP_ROBOT_CONFIG,
            // Mirror path when the robot is on the red alliance (the robot starts on the opposite side of the field)
            { DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red) },
            this
        )

    }

    override fun periodic() {
        val previousGyro = inputs.gyroRotation.toRotation2d().angle

        io.updateInputs(inputs)
        Logger.processInputs("Drivetrain", inputs)

        gyroRate = (inputs.gyroRotation.toRotation2d().angle - previousGyro) / Seconds.of(Robot.period)

        // Update absolute pose sensors and add their measurements to the pose estimator
        for ((name, ioPair) in absolutePoseIOs) {
            val (sensorIO, inputs) = ioPair

            sensorIO.updateInputs(inputs)
            Logger.processInputs("Drivetrain/Absolute Pose/$name", inputs)

            inputs.measurement?.let {
                poseEstimator.addAbsolutePoseMeasurement(it)
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Measurement", it)
                Logger.recordOutput("Drivetrain/Last Added Pose", it.pose)
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Pose", it.pose)
            }
        }

        // Use the new measurements to update the pose estimator
        poseEstimator.update(
            inputs.gyroRotation.toRotation2d(),
            inputs.measuredPositions.toTypedArray()
        )
        Logger.recordOutput("Drivetrain/Estimated Pose", estimatedPose)
        Logger.recordOutput("Drivetrain/Chassis Speeds", measuredChassisSpeeds)
    }

    /** The desired speeds and angles of the swerve modules. */
    private var desiredModuleStates
        get() = io.desiredStates
        set(value) {
            synchronized(this) {
                val stateArr = value.toTypedArray()
                SwerveDriveKinematics.desaturateWheelSpeeds(stateArr, FREE_SPEED)

                io.desiredStates = PerCorner.fromConventionalArray(stateArr)
                Logger.recordOutput("Drivetrain/Desired States", *stateArr)
            }
        }

    /**
     * The current speed of chassis relative to the ground,
     * assuming that the wheels have perfect traction with the ground.
     */
    val measuredChassisSpeeds get() = kinematics.cornerStatesToChassisSpeeds(inputs.measuredStates)

    /**
     * The chassis speeds that the drivetrain is attempting to move at.
     *
     * Note that the speeds are relative to the chassis, not the field.
     */
    private var desiredChassisSpeeds
        get() = kinematics.cornerStatesToChassisSpeeds(desiredModuleStates)
        set(value) {
            val discretized = ChassisSpeeds.discretize(value, Robot.period)
            desiredModuleStates = kinematics.toCornerSwerveModuleStates(discretized)
        }

    /** The estimated pose of the robot on the field, using the yaw value measured by the gyro. */
    var estimatedPose: Pose2d
        get() {
            val estimated = poseEstimator.estimatedPosition
            return Pose2d(
                estimated.x,
                estimated.y,
                inputs.gyroRotation.toRotation2d(),
            )
        }
        private set(value) = poseEstimator.resetPosition(
            inputs.gyroRotation.toRotation2d(),
            inputs.measuredPositions.toTypedArray(),
            value
        )

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType(ElasticWidgets.SwerveDrive.widgetName)
        builder.addDoubleProperty("Robot Angle", { inputs.gyroRotation.toRotation2d().radians }, null)

        builder.addDoubleProperty("Front Left Angle", { io.modules.frontLeft.state.angle.radians }, null)
        builder.addDoubleProperty("Front Left Velocity", { io.modules.frontLeft.state.speedMetersPerSecond }, null)

        builder.addDoubleProperty("Front Right Angle", { io.modules.frontRight.state.angle.radians }, null)
        builder.addDoubleProperty("Front Right Velocity", { io.modules.frontRight.state.speedMetersPerSecond }, null)

        builder.addDoubleProperty("Back Left Angle", { io.modules.backLeft.state.angle.radians }, null)
        builder.addDoubleProperty("Back Left Velocity", { io.modules.backLeft.state.speedMetersPerSecond }, null)

        builder.addDoubleProperty("Back Right Angle", { io.modules.backLeft.state.angle.radians }, null)
        builder.addDoubleProperty("Back Right Velocity", { io.modules.backLeft.state.speedMetersPerSecond }, null)
    }

    private fun isInDeadband(translation: Translation2d) =
        abs(translation.x) < JOYSTICK_DEADBAND && abs(translation.y) < JOYSTICK_DEADBAND

    private fun drive(translationInput: Translation2d, rotationInput: Translation2d) {
        if (isInDeadband(translationInput) && isInDeadband(rotationInput)) {
            // No joystick input - stop moving!
            desiredModuleStates = BRAKE_POSITION
        } else {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translationInput.x * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                translationInput.y * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                -rotationInput.y * TAU * ROTATION_SENSITIVITY,
                inputs.gyroRotation.toRotation2d()
            )
        }
    }

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run {
            // Directly accessing Joystick.x/y gives inverted values - use a `Translation2d` instead.
            drive(translationJoystick.translation2d, rotationJoystick.translation2d)
        }

    @Suppress("unused")
    fun driveWithController(controller: CommandXboxController): Command =
        run {
            val translationInput = Translation2d(controller.leftX, controller.leftY)
            val rotationInput = Translation2d(controller.rightX, controller.rightY)

            drive(translationInput, rotationInput)
        }

    private val rotationPIDController = PIDController(Constants.ROTATION_PID_GAINS).apply {
        enableContinuousInput(0.0, TAU)
    }

    @Suppress("unused")
    fun driveAlignedTo(translationJoystick: Joystick, targetGetter: () -> Translation2d): Command {
        return runEnd({
            val target = targetGetter()

            Logger.recordOutput("Drivetrain/Auto-align Target", target)
            val translationInput = if (abs(translationJoystick.x) > JOYSTICK_DEADBAND
                || abs(translationJoystick.y) > JOYSTICK_DEADBAND
            ) {
                Translation2d(-translationJoystick.y, -translationJoystick.x)
            } else {
                Translation2d()
            }
            val magnitude = rotationPIDController.calculate(
                target.minus(estimatedPose.translation).angle.radians - (TAU / 2),
                estimatedPose.rotation.radians
            )

            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translationInput.x * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                translationInput.y * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                -magnitude,
                inputs.gyroRotation.toRotation2d()
            )
        }, {
            // Might be worth testing this but AdvantageScope seems to ignore `null`s
            Logger.recordOutput("Drivetrain/Auto-align Target", Translation2d())
        })
    }

    fun zeroGyro() {
        // Tell the gyro that the robot is facing the other alliance.
        val zeroPos = when (DriverStation.getAlliance().getOrNull()) {
            DriverStation.Alliance.Blue -> Rotation3d()
            else -> Rotation3d(0.0, 0.0, PI)
        }
        io.setGyro(zeroPos)
    }


    internal object Constants {
        // Translation/rotation coefficient for teleoperated driver controls
        /** Unit: Percent of max robot speed */
        const val TRANSLATION_SENSITIVITY = 0.5

        /** Unit: Rotations per second */
        const val ROTATION_SENSITIVITY = 0.4

        private val WHEEL_BASE: Double = Units.inchesToMeters(30.0)
        private val TRACK_WIDTH: Double = Units.inchesToMeters(28.0)

        const val JOYSTICK_DEADBAND = 0.15

        val MODULE_POSITIONS =
            PerCorner(
                frontLeft =
                Pose2d(
                    Translation2d(WHEEL_BASE, TRACK_WIDTH) / 2.0,
                    Rotation2d.fromDegrees(0.0)
                ),
                frontRight =
                Pose2d(
                    Translation2d(WHEEL_BASE, -TRACK_WIDTH) / 2.0,
                    Rotation2d.fromDegrees(270.0)
                ),
                backLeft =
                Pose2d(
                    Translation2d(-WHEEL_BASE, TRACK_WIDTH) / 2.0,
                    Rotation2d.fromDegrees(90.0)
                ),
                backRight =
                Pose2d(
                    Translation2d(-WHEEL_BASE, -TRACK_WIDTH) / 2.0,
                    Rotation2d.fromDegrees(180.0)
                ),
            )

        // Chassis Control
        val FREE_SPEED = MetersPerSecond.of(8.132)!!
//        private val ROTATION_SPEED = RadiansPerSecond.of(14.604)!!

        val ROTATION_PID_GAINS = PIDGains(3.0, 0.0, 0.4)

//        // Pathing
//        val DEFAULT_PATHING_CONSTRAINTS =
//            PathConstraints(FREE_SPEED.baseUnitMagnitude(), 3.879, ROTATION_SPEED.baseUnitMagnitude(), 24.961)
        // FIXME: Update for 2025
        val PP_ROBOT_CONFIG_COMP = RobotConfig(
            Pounds.of(120.0), // FIXME: Placeholder
            KilogramSquareMeters.of(0.0), // FIXME: Placeholder
            ModuleConfig(
                WHEEL_RADIUS,
                FREE_SPEED,
                1.0, // FIXME: Placeholder
                DCMotor.getKrakenX60(1),
                DRIVING_CURRENT_LIMIT,
                1
            ),
            *MODULE_POSITIONS.map {
                it.translation
            }.toTypedArray()
        )

        val PP_ROBOT_CONFIG_PROTOTYPE = RobotConfig(
            Pounds.of(120.0), // FIXME: Placeholder
            KilogramSquareMeters.of(0.0), // FIXME: Placeholder
            ModuleConfig(
                WHEEL_RADIUS,
                NEO_DRIVING_FREE_SPEED,
                1.0, // FIXME: Placeholder
                DCMotor.getNEO(1),
                DRIVING_CURRENT_LIMIT,
                1
            ),
            *MODULE_POSITIONS.map {
                it.translation
            }.toTypedArray()
        )

        val PP_ROBOT_CONFIG = when(Robot.model) {
            Robot.Model.SIMULATION -> PP_ROBOT_CONFIG_COMP
            Robot.Model.COMPETITION -> PP_ROBOT_CONFIG_COMP
            Robot.Model.PROTOTYPE -> PP_ROBOT_CONFIG_PROTOTYPE
        }



        // CAN IDs
        val KRAKEN_MODULE_CAN_IDS =
            PerCorner(
                frontLeft =
                    Pair(
                        CTREDeviceId.FrontLeftDrivingMotor,
                        REVMotorControllerId.FrontLeftTurningMotor
                    ),
                frontRight =
                    Pair(
                        CTREDeviceId.FrontRightDrivingMotor,
                        REVMotorControllerId.FrontRightTurningMotor
                    ),
                backLeft =
                    Pair(
                        CTREDeviceId.BackLeftDrivingMotor,
                        REVMotorControllerId.BackLeftTurningMotor
                    ),
                backRight =
                    Pair(
                        CTREDeviceId.BackRightDrivingMotor,
                        REVMotorControllerId.BackRightTurningMotor
                    ),
            )

        internal val MODULE_CAN_IDS_PRACTICE =
            PerCorner(
                frontLeft =
                Pair(
                    REVMotorControllerId.FrontLeftDrivingMotor,
                    REVMotorControllerId.FrontLeftTurningMotor
                ),
                frontRight =
                Pair(
                    REVMotorControllerId.FrontRightDrivingMotor,
                    REVMotorControllerId.FrontRightTurningMotor
                ),
                backLeft =
                    Pair(
                        REVMotorControllerId.BackLeftDrivingMotor,
                        REVMotorControllerId.BackLeftTurningMotor
                    ),
                backRight =
                Pair(
                    REVMotorControllerId.BackRightDrivingMotor,
                    REVMotorControllerId.BackRightTurningMotor
                ),
            )

        /** A position with the modules radiating outwards from the center of the robot, preventing movement. */
        val BRAKE_POSITION = MODULE_POSITIONS.map { position -> SwerveModuleState(0.0, position.translation.angle) }
    }
}
