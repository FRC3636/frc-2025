package com.frcteam3636.frc2025.subsystems.drivetrain

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.BRAKE_POSITION
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.DEFAULT_PATHING_CONSTRAINTS
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.FREE_SPEED
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.JOYSTICK_DEADBAND
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.ROTATION_PID_GAINS
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.ROTATION_SENSITIVITY
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain.Constants.TRANSLATION_SENSITIVITY
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.*
import com.frcteam3636.frc2025.utils.ElasticWidgets
import com.frcteam3636.frc2025.utils.fieldRelativeTranslation2d
import com.frcteam3636.frc2025.utils.math.*
import com.frcteam3636.frc2025.utils.swerve.*
import com.frcteam3636.frc2025.utils.translation2d
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathfindingCommand
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.pathfinding.Pathfinding
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import java.util.*
import kotlin.jvm.optionals.getOrDefault
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.withSign

/** A singleton object representing the drivetrain. */
object Drivetrain : Subsystem, Sendable {
    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> DrivetrainIOSim()
        Robot.Model.COMPETITION -> DrivetrainIOReal.fromKrakenSwerve()
        Robot.Model.PROTOTYPE -> DrivetrainIOReal.fromNeoSwerve()
    }
    val inputs = LoggedDrivetrainInputs()

    private val questNavInactiveAlert = Alert("QuestNav localizer is not active.", Alert.AlertType.kInfo)

    //    private val questNavLocalizer = QuestNavLocalizer(Constants.QUESTNAV_DEVICE_OFFSET)
    private val questNavInputs = LoggedQuestNavInputs()
    private var questNavCalibrated = false

    var currentTargetSelection: ReefBranchSide = ReefBranchSide.Right

    private val mt2Algo = LimelightAlgorithm.MegaTag2({
        poseEstimator.estimatedPosition.rotation
    }, {
        inputs.gyroVelocity
    })

    private val absolutePoseIOs = when (Robot.model) {
        Robot.Model.SIMULATION -> mapOf(
            "Limelight" to CameraSimPoseProvider("limelight", Transform3d()),
        )

        else -> mapOf(
            "Limelight Right" to LimelightPoseProvider(
                "limelight-right",
                algorithm = mt2Algo
            ),
            "Limelight Left" to LimelightPoseProvider(
                "limelight-left",
                algorithm = mt2Algo
            ),
//            "Limelight Rear" to LimelightPoseProvider(
//                "limelight-rear",
//                algorithm = mt2Algo
//            ),
        )
    }.mapValues { Pair(it.value, AbsolutePoseProviderInputs()) }

    /** Helper for converting a desired drivetrain velocity into the speeds and angles for each swerve module */
    private val kinematics =
        SwerveDriveKinematics(
            *Constants.MODULE_POSITIONS
                .map { it.translation }
                .toTypedArray()
        )

    /** Helper for estimating the location of the drivetrain on the field */
    private val poseEstimator =
        SwerveDrivePoseEstimator(
            kinematics, // swerve drive kinematics
            inputs.gyroRotation, // initial gyro rotation
            inputs.measuredPositions.toTypedArray(), // initial module positions
            Pose2d(), // initial pose
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5.0)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10.0))
        )

    /** Whether every sensor used for pose estimation is connected. */
    val allPoseProvidersConnected
        get() = absolutePoseIOs.values.all { it.second.connected }

    val isMoving: Boolean
        get() {
            val speeds = measuredChassisSpeeds
            val translationalSpeed = speeds.translation2dPerSecond.norm.metersPerSecond
            return translationalSpeed < 0.5.metersPerSecond
                    && speeds.angularVelocity < 0.5.rotationsPerSecond
        }


    init {
        Pathfinding.setPathfinder(
            LocalADStarAK()
        )

        AutoBuilder.configure(
            this::estimatedPose,
            this::estimatedPose::set,
            this::measuredChassisSpeeds,
            this::desiredChassisSpeeds::set,
            PPHolonomicDriveController(
                when (Robot.model) {
                    Robot.Model.SIMULATION -> DRIVING_PID_GAINS_TALON
                    Robot.Model.COMPETITION -> DRIVING_PID_GAINS_TALON
                    Robot.Model.PROTOTYPE -> DRIVING_PID_GAINS_NEO
                }.toPPLib(),
                ROTATION_PID_GAINS.toPPLib()
            ),
            RobotConfig.fromGUISettings(),
            // Mirror path when the robot is on the red alliance (the robot starts on the opposite side of the field)
            { DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red) },
            this
        )

        if (Robot.model != Robot.Model.SIMULATION) {
            PathfindingCommand.warmupCommand().schedule()
        }

        if (io is DrivetrainIOSim) {
            poseEstimator.resetPose(io.swerveDriveSimulation.simulatedDriveTrainPose)
            io.registerPoseProviders(absolutePoseIOs.values.map { it.first })
        }

        BargeTargetZone.RED.log("Drivetrain/BargeTargetZone/Red")
        BargeTargetZone.BLUE.log("Drivetrain/BargeTargetZone/Blue")
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Drivetrain", inputs)

        // Update absolute pose sensors and add their measurements to the pose estimator
        for ((name, ioPair) in absolutePoseIOs) {
            val (sensorIO, inputs) = ioPair

            sensorIO.updateInputs(inputs)
            Logger.processInputs("Drivetrain/Absolute Pose/$name", inputs)

            Logger.recordOutput("Drivetrain/Absolute Pose/$name/Has Measurement", inputs.measurement != null)
            inputs.measurement?.let {
                poseEstimator.addAbsolutePoseMeasurement(it)
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Measurement", it)
                Logger.recordOutput("Drivetrain/Last Added Pose", it.pose)
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Pose", it.pose)
            }
        }

        // Use the new measurements to update the pose estimator
        poseEstimator.update(
            inputs.gyroRotation,
            inputs.measuredPositions.toTypedArray()
        )

//        questNavLocalizer.updateInputs(questNavInputs)
//        Logger.processInputs("Drivetrain/QuestNav", questNavInputs)
//        updateQuestNavOrigin()

//        Logger.recordOutput("Drivetrain/QuestNav/Calibrated", questNavCalibrated)
        Logger.recordOutput("Drivetrain/Pose Estimator/Estimated Pose", poseEstimator.estimatedPosition)
        Logger.recordOutput("Drivetrain/Estimated Pose", estimatedPose)
        Logger.recordOutput("Drivetrain/Chassis Speeds", measuredChassisSpeeds)
        Logger.recordOutput("Drivetrain/Localizer", localizer.name)
        Logger.recordOutput("Drivetrain/Desired Chassis Speeds", desiredChassisSpeeds)
//        questNavInactiveAlert.set(localizer != Localizer.QuestNav)

        Logger.recordOutput(
            "Drivetrain/TagPoses", *FIELD_LAYOUT.tags
                .filter { tag ->
                    absolutePoseIOs.values.any { it.second.observedTags.contains(tag.ID) }
                }
                .map { it.pose }
                .toTypedArray())
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

    val localizer: Localizer
        get() = if (questNavCalibrated && questNavInputs.connected) {
            Localizer.QuestNav
        } else {
            Localizer.PoseEstimator
        }

    /** The estimated pose of the robot on the field, using the yaw value measured by the gyro. */
    var estimatedPose: Pose2d
        get() {
            val estimated = when (localizer) {
                Localizer.QuestNav -> questNavInputs.pose
                Localizer.PoseEstimator -> poseEstimator.estimatedPosition
            }

            return estimated
        }
        private set(value) {
            poseEstimator.resetPosition(
                inputs.gyroRotation,
                inputs.measuredPositions.toTypedArray(),
                value
            )
        }

    /**
     * Update the QuestNav pose to keep its origin correct.
     */
//    private fun updateQuestNavOrigin() {
//        val hasHighQualityData = absolutePoseIOs.values.any {
//            it.second.measurement != null
//        }
//        if (!hasHighQualityData || isMoving) return
//        questNavLocalizer.resetPose(poseEstimator.estimatedPosition)
//        questNavCalibrated = true
//    }

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType(ElasticWidgets.SwerveDrive.widgetName)
        builder.addDoubleProperty("Robot Angle", { estimatedPose.rotation.radians }, null)

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
                calculateInputCurve(translationInput.x) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                calculateInputCurve(translationInput.y) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                rotationInput.y * TAU * ROTATION_SENSITIVITY,
                estimatedPose.rotation
            )
        }
    }

    private fun calculateInputCurve(input: Double): Double {
        val exponent = 1.7

        return input.absoluteValue.pow(exponent).withSign(input)
    }

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run {
            // Directly accessing Joystick.x/y gives inverted values - use a `Translation2d` instead.
            drive(translationJoystick.fieldRelativeTranslation2d, rotationJoystick.translation2d)
        }

    @Suppress("unused")
    fun driveWithController(controller: CommandXboxController): Command =
        run {
            val translationInput = Translation2d(controller.leftX, controller.leftY)
            val rotationInput = Translation2d(controller.rightX, controller.rightY)

            drive(translationInput, rotationInput)
        }

    private val rotationPIDController = PIDController(ROTATION_PID_GAINS).apply {
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
                estimatedPose.rotation
            )
        }, {
            // Might be worth testing this but AdvantageScope seems to ignore `null`s
            Logger.recordOutput("Drivetrain/Auto-align Target", Translation2d())
        })
    }

    fun alignToClosestPOI(sideOverride: ReefBranchSide? = null, usePathfinding: Boolean = true) =
        alignToTarget(usePathfinding) {
            AprilTagTarget.currentAllianceTargets.asIterable()
                .closestTargetToPoseWithSelection(estimatedPose, sideOverride ?: currentTargetSelection).pose
        }

    fun alignToBarge(usePathfinding: Boolean = true) = alignToTarget(usePathfinding) {
        BargeTarget.closestTo(
            estimatedPose.translation,
            DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue)
        ).pose
    }

    fun alignToReefAlgae(usePathfinding: Boolean = true) = alignToTarget(usePathfinding) {
        AprilTagTarget.currentAllianceReefAlgaeTargets
            .asIterable()
            .closestToPose(estimatedPose)
            .pose
    }

    /** Use PID only to drive to left human player station */
    fun alignToLeftStation() = alignToTarget(usePathfinding = false) {
        AprilTagTarget.currentAllianceLeftStation.pose
    }

    /** Use PID only to drive to right human player station */
    fun alignToRightStation() = alignToTarget(usePathfinding = false) {
        AprilTagTarget.currentAllianceRightStation.pose
    }

    private val alignTranslationController = PIDController(Constants.ALIGN_TRANSLATION_PID_GAINS)
    private val alignRotationController = PIDController(Constants.ALIGN_ROTATION_PID_GAINS)

    /**
     * Drive to a pose on the field.
     *
     * @param usePathfinding - If enabled, uses PathPlanner to drive a long distance without colliding with anything.
     * @param target - A function that returns the desired pose (called each time the command starts)
     */
    fun alignToTarget(usePathfinding: Boolean = true, target: () -> Pose2d): Command = defer {
        // If true, controls rotation with PID
        val enableRotationControl = Preferences.getBoolean("AlignUseRotationControl", true)

        // If true, ends the command when in place
        val enableEndCondition = Preferences.getBoolean("AlignUseEndCondition", true)

        val target = target()
        Logger.recordOutput("Drivetrain/Auto-align Target", target)

        val commands = mutableListOf<Command>()

        if (usePathfinding) {
            commands.add(AutoBuilder.pathfindToPose(target, DEFAULT_PATHING_CONSTRAINTS, 1.0.metersPerSecond))
        }

        commands.addAll(
            arrayOf(
                runOnce {
                    alignTranslationController.reset()
                    alignRotationController.reset()
                    Logger.recordOutput("/Drivetrain/Align-Running", true)
                },
                runEnd({
                    val relativePose = estimatedPose.relativeTo(target)
                    val distanceToTarget = relativePose.translation.norm
                    val angleToTarget = relativePose.translation.angle
                    val output = alignTranslationController.calculate(distanceToTarget, 0.0)
                    val desiredSpeed = Translation2d(output, angleToTarget)

                    val rotation = if (enableRotationControl) {
                        alignRotationController
                            .calculate(
                                relativePose.rotation.degrees,
                                0.0
                            )
                            .degreesPerSecond
                    } else {
                        0.degreesPerSecond
                    }

                    val chassisSpeeds = ChassisSpeeds(
                        desiredSpeed.x.metersPerSecond,
                        desiredSpeed.y.metersPerSecond,
                        rotation,
                    )
                    Logger.recordOutput("/Drivetrain/Auto-align Chassis Speeds", chassisSpeeds)
                    desiredChassisSpeeds = chassisSpeeds
                }, {
                    desiredModuleStates = BRAKE_POSITION
                    Logger.recordOutput("/Drivetrain/Align-Running", false)
                })
            )
        )

        Commands.sequence(*commands.toTypedArray())
            .until {
                if (enableEndCondition) {
                    val relativePose = estimatedPose.relativeTo(target)

                    relativePose.translation.norm < 0.3.inches.inMeters() // Translation
                            && abs(relativePose.rotation.degrees) < 1.5 // Rotation
                            && measuredChassisSpeeds.translation2dPerSecond.norm < 0.25 // Speed
                } else {
                    false
                }
            }
    }

    fun zeroGyro(isReversed: Boolean = false) {
        // Tell the gyro that the robot is facing the other alliance.
        var zeroPos = when (DriverStation.getAlliance().getOrNull()) {
            DriverStation.Alliance.Red -> Rotation2d.k180deg
            else -> Rotation2d.kZero
        }
        if (isReversed) {
            zeroPos += Rotation2d.k180deg
        }
        estimatedPose = Pose2d(estimatedPose.translation, zeroPos)
//        io.setGyro(zeroPos)
    }

    var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            0.5.voltsPerSecond, 2.volts, null, {
                SignalLogger.writeString("state", it.toString())
            }), SysIdRoutine.Mechanism(
            io::runCharacterization,
            null,
            this,
        )
    )

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts)
    }.withTimeout(1.0).andThen(sysID.quasistatic(direction))!!

    fun sysIdDynamic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts)
    }.withTimeout(1.0).andThen(sysID.dynamic(direction))!!

    internal object Constants {
        // Translation/rotation coefficient for teleoperated driver controls
        /** Unit: Percent of max robot speed */
        const val TRANSLATION_SENSITIVITY = 1.0 // FIXME: Increase

        /** Unit: Rotations per second */
        const val ROTATION_SENSITIVITY = 0.8

        val WHEEL_BASE = 30.inches
        val TRACK_WIDTH = 28.inches

        val BUMPER_WIDTH = 33.5.inches
        val BUMPER_LENGTH = 35.5.inches

        const val JOYSTICK_DEADBAND = 0.075

        val MODULE_POSITIONS = PerCorner(
            frontLeft = Pose2d(
                Translation2d(WHEEL_BASE, TRACK_WIDTH) / 2.0, Rotation2d.fromDegrees(0.0)
            ),
            frontRight = Pose2d(
                Translation2d(WHEEL_BASE, -TRACK_WIDTH) / 2.0, Rotation2d.fromDegrees(270.0)
            ),
            backLeft = Pose2d(
                Translation2d(-WHEEL_BASE, TRACK_WIDTH) / 2.0, Rotation2d.fromDegrees(90.0)
            ),
            backRight = Pose2d(
                Translation2d(-WHEEL_BASE, -TRACK_WIDTH) / 2.0, Rotation2d.fromDegrees(180.0)
            ),
        )

        // Chassis Control
        val FREE_SPEED = 5.5.metersPerSecond
        private val ROTATION_SPEED = 14.604.radiansPerSecond

        val ROTATION_PID_GAINS = PIDGains(3.0, 0.0, 0.4)

        //        // Pathing
        val DEFAULT_PATHING_CONSTRAINTS =
            PathConstraints(
                FREE_SPEED.baseUnitMagnitude() * 2,
                (3.879 * 1.5) * 2.0,
                ROTATION_SPEED.baseUnitMagnitude(),
                24.961
            )

        // FIXME: Update for 2025
        val PP_ROBOT_CONFIG_COMP = RobotConfig(
            120.0.pounds, // FIXME: Placeholder
            0.kilogramSquareMeters, // FIXME: Placeholder
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
            120.pounds, // FIXME: Placeholder
            0.kilogramSquareMeters, // FIXME: Placeholder
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

        val PP_ROBOT_CONFIG = when (Robot.model) {
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

        val QUESTNAV_DEVICE_OFFSET = Transform2d(
            // TODO: find these constants
            0.inches,
            0.inches,
            Rotation2d(0.degrees)
        )

        val ALIGN_TRANSLATION_PID_GAINS = PIDGains(7.0)
        val ALIGN_ROTATION_PID_GAINS = PIDGains(0.5)
    }

    enum class Localizer {
        QuestNav,
        PoseEstimator,
    }
}
