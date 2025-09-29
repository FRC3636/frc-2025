package com.frcteam3636.frc2025.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
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
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.AutoMode
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.*
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.utils.fieldRelativeTranslation2d
import com.frcteam3636.frc2025.utils.math.*
import com.frcteam3636.frc2025.utils.swerve.PerCorner
import com.frcteam3636.frc2025.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.frc2025.utils.swerve.toCornerSwerveModuleStates
import com.frcteam3636.frc2025.utils.swerve.translation2dPerSecond
import com.frcteam3636.frc2025.utils.translation2d
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.FollowPathCommand
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.*
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.IntegerPublisher
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
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
object Drivetrain : Subsystem {
    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> DrivetrainIOSim()
        Robot.Model.COMPETITION -> DrivetrainIOReal.fromKrakenSwerve()
        Robot.Model.PROTOTYPE -> DrivetrainIOReal.fromNeoSwerve()
    }
    val inputs = LoggedDrivetrainInputs()

    var currentTargetSelection: ReefBranchSide = ReefBranchSide.Right

    private val alignPositionPublisher = NetworkTableInstance.getDefault()
        .getDoubleArrayTopic("RGB/Auto Align/Position Relative to Align Target")
        .publish()
    val alignStatePublisher: IntegerPublisher = NetworkTableInstance.getDefault()
        .getIntegerTopic("RGB/Movement State")
        .publish()
        .apply {
            setDefault(AlignState.NotRunning.raw)
        }

    var tagsVisible = false

    enum class AlignState(val raw: Long) {
        NotRunning(0),
        AlignPathfinding(1),
        Aligning(2),
        Success(3),
    }

    val mt2Algo = LimelightAlgorithm.MegaTag2({
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
                mt2Algo
            ),
            "Limelight Left" to LimelightPoseProvider(
                "limelight-left",
                mt2Algo
            )
        )
    }.mapValues { Pair(it.value, AbsolutePoseProviderInputs()) }

    val limelightsConnected: Boolean
        get() = absolutePoseIOs.values.all { it.second.connected }

    /** Helper for converting a desired drivetrain velocity into the speeds and angles for each swerve module */
    private val kinematics =
        SwerveDriveKinematics(
            *Constants.MODULE_POSITIONS
                .map { it.translation }
                .toTypedArray()
        )

    /** Helper for estimating the location of the drivetrain on the field */
    val poseEstimator =
        SwerveDrivePoseEstimator(
            kinematics, // swerve drive kinematics
            inputs.gyroRotation, // initial gyro rotation
            inputs.measuredPositions.toTypedArray(), // initial module positions
            Pose2d(), // initial pose
            VecBuilder.fill(0.02, 0.02, 0.005),
            // Overwrite each measurement
            VecBuilder.fill(0.0, 0.0, 0.0)
        )


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
                    Robot.Model.COMPETITION -> Constants.ALIGN_TRANSLATION_PID_GAINS
                    Robot.Model.PROTOTYPE -> DRIVING_PID_GAINS_NEO
                }.toPPLib(),
                Constants.ALIGN_ROTATION_PID_GAINS.toPPLib()
            ),
            RobotConfig.fromGUISettings(),
            // Mirror path when the robot is on the red alliance (the robot starts on the opposite side of the field)
            {
                @Suppress("IDENTITY_SENSITIVE_OPERATIONS_WITH_VALUE_TYPE")
                DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)
            },
            this
        )

        if (Robot.model != Robot.Model.SIMULATION) {
            FollowPathCommand.warmupCommand().schedule()
        }

        if (io is DrivetrainIOSim) {
            poseEstimator.resetPose(io.swerveDriveSimulation.simulatedDriveTrainPose)
            io.registerPoseProviders(absolutePoseIOs.values.map { it.first })
        }

        BargeTargetZone.RED.log("Drivetrain/BargeTargetZone/Red")
        BargeTargetZone.BLUE.log("Drivetrain/BargeTargetZone/Blue")
    }

    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return io.getStatusSignals()
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Drivetrain", inputs)
        tagsVisible = false

        // Update absolute pose sensors and add their measurements to the pose estimator
        for ((name, ioPair) in absolutePoseIOs) {
            val (sensorIO, inputs) = ioPair

            sensorIO.updateInputs(inputs)
            Logger.processInputs("Drivetrain/Absolute Pose/$name", inputs)

            Logger.recordOutput("Drivetrain/Absolute Pose/$name/Has Measurement", inputs.measurement != null)
            Logger.recordOutput("Drivetrain/Absolute Pose/$name/Connected", inputs.connected)
            if (inputs.observedTags.isNotEmpty())
                tagsVisible = true

            inputs.measurement?.let {
                poseEstimator.addAbsolutePoseMeasurement(it)
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Measurement", it)
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Pose", it.pose)
            }
        }

        // Use the new measurements to update the pose estimator
        poseEstimator.update(
            inputs.gyroRotation,
            inputs.measuredPositions.toTypedArray()
        )

        Logger.recordOutput("Drivetrain/Pose Estimator/Estimated Pose", poseEstimator.estimatedPosition)
        Logger.recordOutput("Drivetrain/Estimated Pose", estimatedPose)
        Logger.recordOutput("Drivetrain/Chassis Speeds", measuredChassisSpeeds)
        Logger.recordOutput("Drivetrain/Desired Chassis Speeds", desiredChassisSpeeds)

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


    /** The estimated pose of the robot on the field, using the yaw value measured by the gyro. */
    var estimatedPose: Pose2d
        get() {
            return poseEstimator.estimatedPosition
        }
        private set(value) {
            poseEstimator.resetPosition(
                inputs.gyroRotation,
                inputs.measuredPositions.toTypedArray(),
                value
            )
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

    fun alignToClosestPOI(
        sideOverride: ReefBranchSide? = null,
        usePathfinding: Boolean = true,
        raiseElevator: Boolean = false,
        endConditionTimeout: Double = 0.75
    ) =
        alignToTarget(usePathfinding, raiseElevator, endConditionTimeout = endConditionTimeout) {
            AprilTagTarget.currentAllianceTargets.asIterable()
                .closestTargetToPoseWithSelection(estimatedPose, sideOverride ?: currentTargetSelection).pose
        }

    fun alignToBarge(usePathfinding: Boolean = true) = alignToTarget(usePathfinding) {
        BargeTarget.closestTo(
            estimatedPose.translation,
            DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue)
        ).pose
    }

    fun alignToReefAlgae(usePathfinding: Boolean = true) = alignToTarget(usePathfinding, disableEndConditionOverride = true) {
        AprilTagTarget.currentAllianceReefAlgaeTargets
            .asIterable()
            .closestToPose(estimatedPose)
            .pose
    }


    fun isAtTarget(relativePose: Pose2d): Boolean =
        relativePose.translation.norm < 2.centimeters.inMeters() // Translation
                && Elevator.isAtTarget
                && measuredChassisSpeeds.translation2dPerSecond.norm.metersPerSecond < 0.2.metersPerSecond

    fun driveToPointAllianceRelative(target: Pose2d, constraints: PathConstraints = DEFAULT_PATHING_CONSTRAINTS,
                                     startingPoseHeadingOffset: Rotation2d = Rotation2d.kZero,
                                     targetPoseHeadingOffset: Rotation2d = Rotation2d.kZero): Command {
        return defer {
            var startingPose = estimatedPose
            var heading = (target.translation - estimatedPose.translation).angle
            var updatedTargetPose = target
            if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
                updatedTargetPose = FlippingUtil.flipFieldPose(target)
                heading = (updatedTargetPose.translation - startingPose.translation).angle
            }

            assert(startingPose.translation != updatedTargetPose.translation)

            startingPose = Pose2d(startingPose.translation, heading + startingPoseHeadingOffset)
            val waypoints: List<Waypoint> = PathPlannerPath.waypointsFromPoses(
                startingPose,
                Pose2d(updatedTargetPose.translation, heading + targetPoseHeadingOffset)
            )

            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Middle Point", false)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Slow Zone", false)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Constraint Zone", false)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Updated Target Pose", Pose2d(updatedTargetPose.translation, heading + targetPoseHeadingOffset))
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Updated Starting Pose", startingPose)

            val path = PathPlannerPath(
                waypoints,
                constraints,
                null,
                GoalEndState(0.0, updatedTargetPose.rotation)
            )

            path.preventFlipping = true

            AutoBuilder.followPath(path)
        }
    }
    fun driveToPointAllianceRelativeWithMiddlePoint(target: Pose2d, constraints: PathConstraints = DEFAULT_PATHING_CONSTRAINTS, middlePoint: Pose2d): Command {
        return defer {
            var startingPose = estimatedPose
            var middlePoint = middlePoint
            var headingToMiddlePoint = (middlePoint.translation - startingPose.translation).angle
            var updatedTargetPose = target
            var headingFromMiddlePoint = (updatedTargetPose.translation - middlePoint.translation).angle
            if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
                updatedTargetPose = FlippingUtil.flipFieldPose(target)
                middlePoint = FlippingUtil.flipFieldPose(middlePoint)
                headingFromMiddlePoint = (updatedTargetPose.translation - startingPose.translation).angle
                headingToMiddlePoint = (middlePoint.translation - startingPose.translation).angle
            }

            assert(startingPose.translation != updatedTargetPose.translation)
            assert(startingPose.translation != middlePoint.translation)
            assert(middlePoint .translation != updatedTargetPose.translation)

            startingPose = Pose2d(startingPose.translation, headingToMiddlePoint)
            val waypoints: List<Waypoint> = PathPlannerPath.waypointsFromPoses(
                startingPose,
                Pose2d(middlePoint.translation, headingFromMiddlePoint),
                Pose2d(updatedTargetPose.translation, headingFromMiddlePoint)
            )

            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Middle Point", true)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Slow Zone", false)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Constraint Zone", false)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Updated Target Pose", Pose2d(updatedTargetPose.translation, headingFromMiddlePoint))
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Middle Pose", Pose2d(middlePoint.translation, headingFromMiddlePoint))
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Updated Starting Pose", startingPose)

            val path = PathPlannerPath(
                waypoints,
                constraints,
                null,
                GoalEndState(0.0, updatedTargetPose.rotation)
            )

            path.preventFlipping = true

            AutoBuilder.followPath(path)
        }
    }

    fun driveToPointAllianceRelativeWithSlowZone(target: Pose2d, constraints: PathConstraints = DEFAULT_PATHING_CONSTRAINTS, constraintsSlowZone: PathConstraints = DEFAULT_PATHING_CONSTRAINTS, slowZoneDistanceFromTarget: Distance, slowZoneStartVelocity: LinearVelocity, shouldRaise: Boolean = true, raisePoint: Elevator.Position = Elevator.Position.HighBar): Command {
        return defer {
            assert(slowZoneDistanceFromTarget > 0.0.meters)
            var startingPose = estimatedPose
            var slowZoneStart = target.backup(slowZoneDistanceFromTarget)
            var heading = (slowZoneStart.translation - estimatedPose.translation).angle
            var slowZoneHeading = (target.translation - slowZoneStart.translation).angle
            var updatedTargetPose = target
            if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
                updatedTargetPose = FlippingUtil.flipFieldPose(target)
                slowZoneStart = FlippingUtil.flipFieldPose(slowZoneStart)
                heading = (updatedTargetPose.translation - startingPose.translation).angle
                slowZoneHeading = (updatedTargetPose.translation - slowZoneStart.translation).angle
            }

            assert(startingPose.translation != updatedTargetPose.translation)
            assert(slowZoneStart.translation != updatedTargetPose.translation)
            assert(slowZoneStart.translation != startingPose.translation)

            startingPose = Pose2d(startingPose.translation, heading)
            val firstPathWaypoints: List<Waypoint> = PathPlannerPath.waypointsFromPoses(
                startingPose,
                Pose2d(slowZoneStart.translation, heading)
            )

            val slowZoneWaypoints: List<Waypoint> = PathPlannerPath.waypointsFromPoses(
                Pose2d(slowZoneStart.translation, slowZoneHeading),
                Pose2d(updatedTargetPose.translation, slowZoneHeading)
            )

            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Middle Point", false)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Slow Zone", true)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Constraint Zone", false)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Updated Target Pose", Pose2d(updatedTargetPose.translation, heading))
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Updated Starting Pose", startingPose)

            val firstPath = PathPlannerPath(
                firstPathWaypoints,
                constraints,
                null,
                GoalEndState(slowZoneStartVelocity, updatedTargetPose.rotation)
            )

            firstPath.preventFlipping = true

            val secondPath = PathPlannerPath(
                slowZoneWaypoints,
                constraintsSlowZone,
                null,
                GoalEndState(0.0, updatedTargetPose.rotation)
            )

            secondPath.preventFlipping = true

            Commands.sequence(
                AutoBuilder.followPath(firstPath),
                Commands.parallel(AutoBuilder.followPath(secondPath),
                    Elevator.setTargetHeight(raisePoint).onlyIf { shouldRaise }
                )
            )
        }
    }

    fun driveToPointAllianceRelativeWithSlowConstraintZone(target: Pose2d, constraints: PathConstraints = DEFAULT_PATHING_CONSTRAINTS, constraintsSlowZone: PathConstraints = DEFAULT_PATHING_CONSTRAINTS, slowZoneDistanceFromTarget: Distance, shouldRaise: Boolean = true, raisePoint: Elevator.Position = Elevator.Position.HighBar): Command {
        return defer {
            assert(slowZoneDistanceFromTarget > 0.0.meters)
            var startingPose = estimatedPose
            var heading = (target.translation - estimatedPose.translation).angle
            var updatedTargetPose = target
            if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
                updatedTargetPose = FlippingUtil.flipFieldPose(target)
                heading = (updatedTargetPose.translation - startingPose.translation).angle
            }

            assert(startingPose.translation != updatedTargetPose.translation)

            startingPose = Pose2d(startingPose.translation, heading)
            val waypoints: List<Waypoint> = PathPlannerPath.waypointsFromPoses(
                startingPose,
                Pose2d(updatedTargetPose.translation, heading)
            )

            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Middle Point", false)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Slow Zone", true)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Using Constraint Zone", true)
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Updated Target Pose", Pose2d(updatedTargetPose.translation, heading))
            Logger.recordOutput("Drivetrain/Auto Drive to Point/Updated Starting Pose", startingPose)

            val distanceToTarget = startingPose.translation.getDistance(target.translation).meters
            assert(slowZoneDistanceFromTarget < distanceToTarget)

            val minPosition = 1.0 - (slowZoneDistanceFromTarget.inMeters() / distanceToTarget.inMeters())

            val constraintZone = ConstraintsZone(
                minPosition, 1.0, constraintsSlowZone
            )

            val path = PathPlannerPath(
                waypoints,
                emptyList(),
                emptyList(),
                mutableListOf(constraintZone),
                emptyList(),
                constraints,
                null,
                GoalEndState(0.0, updatedTargetPose.rotation),
                false
            )

            path.preventFlipping = true

            Commands.parallel(
                AutoBuilder.followPath(path),
                Commands.sequence(
                    Commands.waitUntil {
                        estimatedPose.translation.getDistance(updatedTargetPose.translation).meters <= slowZoneDistanceFromTarget
                    },
                    Elevator.setTargetHeight(raisePoint)
                ).onlyIf { shouldRaise }
            )
        }
    }

    /**
     * Drive to a pose on the field.
     *
     * @param usePathfinding - If enabled, uses PathPlanner to drive a long distance without colliding with anything.
     * @param target - A function that returns the desired pose (called each time the command starts)
     */
    fun alignToTarget(
        usePathfinding: Boolean = true,
        raiseElevator: Boolean = false,
        disableEndConditionOverride: Boolean = false,
        endConditionTimeout: Double = 0.75,
        target: () -> Pose2d
    ): Command {
        return defer {
            // If true, ends the command when in place
            val useEndCondition = Preferences.getBoolean("AlignUseEndCondition", true) && !disableEndConditionOverride

            val target = target()
            Logger.recordOutput("Drivetrain/Auto-align Target", target)

            Logger.recordOutput(
                "/Drivetrain/Auto Align/Has Reached Target",
                false
            )

            val commands = mutableListOf<Command>()

            if (usePathfinding) {
                commands.add(Commands.runOnce({
                    alignStatePublisher.set(AlignState.AlignPathfinding.raw)
                }))
                commands.add(AutoBuilder.pathfindToPose(target, DEFAULT_PATHING_CONSTRAINTS, 1.0.metersPerSecond))
            }

            if (raiseElevator) {
                commands.add(
                    Elevator.setTargetHeight(Elevator.Position.HighBar)
                )
            }

            val heading = (target.translation - estimatedPose.translation).angle
            val startingPose = Pose2d(estimatedPose.translation, heading)

            val waypoints: List<Waypoint> = PathPlannerPath.waypointsFromPoses(
                startingPose,
                Pose2d(target.translation, heading)
            )
            
            val constraints = PathConstraints(10.0,3.0, 2 * Math.PI, 4 * Math.PI)

            val constraintZone = ConstraintsZone(
                0.85, 1.0, AutoMode.DEFAULT_AUTO_CONSTRAINTS_SLOW_ZONE
            )

            val path = PathPlannerPath(
                waypoints,
                emptyList(),
                emptyList(),
                mutableListOf(constraintZone),
                emptyList(),
                constraints,
                null,
                GoalEndState(0.0, target.rotation),
                false
            )

            path.preventFlipping = true

            commands.add(Commands.runOnce({
                alignStatePublisher.set(AlignState.Aligning.raw)
            }))
            commands.add(AutoBuilder.followPath(path))

            val endCondition = Trigger {
                val relativePose = estimatedPose.relativeTo(target)
                isAtTarget(relativePose)
            }
                .debounce(endConditionTimeout)

            if (useEndCondition) {
                Commands.sequence(*commands.toTypedArray())
                    .alongWith(
                        Commands.run({
                            val relativePose = estimatedPose.relativeTo(target)

                            alignPositionPublisher.set(
                                doubleArrayOf(
                                    relativePose.x,
                                    relativePose.y,
                                )
                            )
                        })
                    )
                    .finallyDo { ->
                        if (endCondition.asBoolean) {
                            alignStatePublisher.set(AlignState.Success.raw)
                        } else {
                            alignStatePublisher.set(AlignState.NotRunning.raw)
                        }
                    }
                    .until(endCondition)
            } else {
                Commands.sequence(*commands.toTypedArray())
            }
        }
    }

    fun zeroGyro(isReversed: Boolean = false, offset: Rotation2d = Rotation2d.kZero) {
        // Tell the gyro that the robot is facing the other alliance.
        var zeroPos = when (DriverStation.getAlliance().getOrNull()) {
            DriverStation.Alliance.Red -> Rotation2d.k180deg
            else -> Rotation2d.kZero
        }

        if (isReversed) {
            zeroPos += Rotation2d.k180deg
        }

        estimatedPose = Pose2d(estimatedPose.translation, zeroPos + offset)
//        io.setGyro(zeroPos)
    }

    var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            0.5.voltsPerSecond, 2.volts, null
        ) {
            SignalLogger.writeString("state", it.toString())
        }, SysIdRoutine.Mechanism(
            io::runCharacterization,
            null,
            this,
        )
    )

    @Suppress("unused")
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts)
    }.withTimeout(1.0).andThen(sysID.quasistatic(direction))!!

    @Suppress("unused")
    fun sysIdDynamic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts)
    }.withTimeout(1.0).andThen(sysID.dynamic(direction))!!

    internal object Constants {
        // Translation/rotation coefficient for teleoperated driver controls
        /** Unit: Percent of max robot speed */
        const val TRANSLATION_SENSITIVITY = 1.0

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

        val ROTATION_PID_GAINS = PIDGains(3.0, 0.0, 0.4)

        //        // Pathing
        val DEFAULT_PATHING_CONSTRAINTS =
            PathConstraints(
                3.0, 3.0, 2 * Math.PI, 4 * Math.PI
            )

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


        val ALIGN_TRANSLATION_PID_GAINS = PIDGains(5.0)
        val ALIGN_ROTATION_PID_GAINS = PIDGains(5.0)
    }
}
