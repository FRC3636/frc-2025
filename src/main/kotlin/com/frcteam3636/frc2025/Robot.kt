package com.frcteam3636.frc2025

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.StatusSignal
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.poi.ReefBranchSide
import com.frcteam3636.frc2025.subsystems.elevator.Elevator
import com.frcteam3636.frc2025.subsystems.funnel.Funnel
import com.frcteam3636.frc2025.subsystems.manipulator.CoralState
import com.frcteam3636.frc2025.subsystems.manipulator.Manipulator
import com.frcteam3636.frc2025.utils.math.seconds
import com.frcteam3636.frc2025.utils.rumble
import com.frcteam3636.version.BUILD_DATE
import com.frcteam3636.version.DIRTY
import com.frcteam3636.version.GIT_BRANCH
import com.frcteam3636.version.GIT_SHA
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import kotlin.io.path.Path
import kotlin.io.path.exists


/**
 * The VM is configured to automatically run this object (which basically functions as a singleton
 * class), and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. This is written as an object rather than a class since there should only ever be a
 * single instance, and it cannot take any constructor arguments. This makes it a natural fit to be
 * an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also
 * update the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when
 * renaming the object or package, it will get changed everywhere.)
 */
object Robot : LoggedRobot() {
    private val controller = CommandXboxController(2)
    private val joystickLeft = CommandJoystick(0)
    private val joystickRight = CommandJoystick(1)

    @Suppress("unused")
    private val joystickDev = Joystick(3)

    private var autoCommand: Command? = null

    private val rioCANBus = CANBus("rio")
    private val canivore = CANBus("*")

    /** Status signals used to check the health of the robot's hardware */
    val statusSignals = mutableMapOf<String, StatusSignal<*>>()

    override fun robotInit() {
        // Report the use of the Kotlin Language for "FRC Usage Report" statistics
        HAL.report(
            tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version
        )

        SignalLogger.enableAutoLogging(false)

        // Joysticks are likely to be missing in simulation, which usually isn't a problem.
        DriverStation.silenceJoystickConnectionWarning(model != Model.COMPETITION)

        configureAdvantageKit()
        configureSubsystems()
        configureAutos()
        configureBindings()
        configureDashboard()

        Diagnostics.reportLimelightsInBackground(arrayOf("limelight-left", "limelight-right"))

    }

    /** Start logging or pull replay logs from a file */
    private fun configureAdvantageKit() {
        Logger.recordMetadata("Git SHA", GIT_SHA)
        Logger.recordMetadata("Build Date", BUILD_DATE)
        @Suppress("KotlinConstantConditions")
        Logger.recordMetadata("Git Tree Dirty", (DIRTY == 1).toString())
        Logger.recordMetadata("Git Branch", GIT_BRANCH)
        Logger.recordMetadata("Model", model.name)

        if (isReal()) {
            Logger.addDataReceiver(WPILOGWriter()) // Log to a USB stick
            if (!Path("/U").exists()) {
                Alert(
                    "The Log USB drive is not connected to the roboRIO, so a match replay will not be saved. (If convenient, insert it and restart robot code.)",
                    Alert.AlertType.kInfo
                )
                    .set(true)
            }
            Logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
            // Enables power distribution logging
            if (model == Model.COMPETITION) {
                PowerDistribution(
                    1, PowerDistribution.ModuleType.kRev
                )
            } else {
                PowerDistribution(
                    1, PowerDistribution.ModuleType.kCTRE
                )
            }
        } else {
            val logPath = try {
                // Pull the replay log from AdvantageScope (or prompt the user)
                LogFileUtil.findReplayLog()
            } catch (_: java.util.NoSuchElementException) {
                null
            }

            if (logPath == null) {
                // No replay log, so perform physics simulation
                Logger.addDataReceiver(NT4Publisher())
            } else {
                // Replay log exists, so replay data
                setUseTiming(false) // Run as fast as possible
                Logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
                Logger.addDataReceiver(
                    WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
                ) // Save outputs to a new log
            }
        }
        Logger.start() // Start logging! No more data receivers, replay sources, or metadata values may be added.
    }

    /** Start robot subsystems so that their periodic tasks are run */
    private fun configureSubsystems() {
        Drivetrain.register()
        Manipulator.register()
        Elevator.register()
        Funnel.register()
    }

    /** Expose commands for autonomous routines to use and display an auto picker in Shuffleboard. */
    private fun configureAutos() {
//        NamedCommands.registerCommand(
//            "revAim",
//            Commands.parallel(
//                Shooter.Pivot.followMotionProfile(Shooter.Pivot.Target.AIM),
//                Shooter.Flywheels.rev(580.0, 0.0)
//            )
//        )
        NamedCommands.registerCommand(
            "raiseElevatorL4",
            Elevator.setTargetHeight(Elevator.Position.HighBar)
                .andThen(Commands.waitUntil { Elevator.isAtTarget })
        )
        NamedCommands.registerCommand(
            "raiseElevatorL3",
            Elevator.setTargetHeight(Elevator.Position.MidBar)
                .andThen(Commands.waitUntil { Elevator.isAtTarget })
        )
        NamedCommands.registerCommand(
            "raiseElevatorL2",
            Elevator.setTargetHeight(Elevator.Position.LowBar)
                .andThen(Commands.waitUntil { Elevator.isAtTarget })
        )
        NamedCommands.registerCommand(
            "stowElevator",
            Elevator.setTargetHeight(Elevator.Position.Stowed)
        )
        NamedCommands.registerCommand(
            "outtake",
            Manipulator.outtake().withTimeout(0.3.seconds)
        )
        NamedCommands.registerCommand(
            "intake",
            Commands.race(
                Manipulator.intakeAuto(),
                Funnel.intake()
            ).withTimeout(3.0)
        )
        NamedCommands.registerCommand(
            "alignToTarget",
            Drivetrain.alignToClosestPOI(
                sideOverride = ReefBranchSide.Left,
                usePathfinding = false,
                raiseElevator = false,
                endConditionTimeout = 0.5
            )
                .withTimeout(3.5.seconds)
        )
        NamedCommands.registerCommand(
            "alignToTargetRight",
            Drivetrain.alignToClosestPOI(
                sideOverride = ReefBranchSide.Right,
                usePathfinding = false,
                raiseElevator = false,
                endConditionTimeout = 0.5
            )
                .withTimeout(3.5.seconds)
        )
        NamedCommands.registerCommand(
            "raiseElevatorAlgae",
            Elevator.setTargetHeight(Elevator.Position.AlgaeMidBar)
        )
        NamedCommands.registerCommand(
            "alignToReefAlgae",
            Drivetrain.alignToReefAlgae(usePathfinding = false)
                .withTimeout(0.75.seconds)
        )
        NamedCommands.registerCommand(
            "alignToBarge",
            Drivetrain.alignToBarge(usePathfinding = false)
                .withTimeout(0.9.seconds)
        )
        NamedCommands.registerCommand(
            "intakeAlgae",
            Commands.sequence(
                Commands.runOnce({
                    Manipulator.isIntakeRunning = true
                }),
                Manipulator.intakeAlgae(),
            )
        )
        NamedCommands.registerCommand(
            "tossAlgae",
            tossAlgae()
        )
        NamedCommands.registerCommand(
            "alignToStation",
            Commands.parallel(
                Drivetrain.alignToClosestPOI(sideOverride = ReefBranchSide.Right, usePathfinding = false)
                    .withTimeout(1.seconds),
                Elevator.setTargetHeight(Elevator.Position.HighBar),
            )
        )
        NamedCommands.registerCommand(
            "waitForIntake",
            Commands.race(
                Commands.waitUntil {
                    Manipulator.coralState != CoralState.NONE
                },
                Commands.waitSeconds(1.0)
            )
        )
    }

    private fun tossAlgae(): Command = Commands.sequence(
        Commands.parallel(
            Elevator.setTargetHeightAlgae(Elevator.Position.HighBar),
            Commands.sequence(
                Commands.runOnce({
                    Manipulator.isIntakeRunning = true
                }),
                Commands.race(
                    Manipulator.intakeAlgae(),
                    Commands.waitSeconds(0.4),
                ),
                Commands.runOnce({
                    Manipulator.isIntakeRunning = false
                }),
                Manipulator.outtakeAlgae().withTimeout(0.75),
            )
        ),
        Elevator.setTargetHeight(Elevator.Position.Stowed)
    )

    /** Configure which commands each joystick button triggers. */
    private fun configureBindings() {
        Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks(joystickLeft.hid, joystickRight.hid)
        Manipulator.defaultCommand = Manipulator.idle()

        joystickRight.button(3).onTrue(Commands.runOnce({
            println("Setting desired target node to left branch.")
            Drivetrain.currentTargetSelection = ReefBranchSide.Left
        }))

        joystickRight.button(4).onTrue(Commands.runOnce({
            println("Setting desired target node to right branch.")
            Drivetrain.currentTargetSelection = ReefBranchSide.Right
        }))

        joystickLeft.button(1).whileTrue(Drivetrain.alignToClosestPOI(endConditionTimeout = 0.5))
        joystickRight.povUp().whileTrue(
            Drivetrain.alignToReefAlgae()
        )
//        joystickLeft.button(1).whileTrue(Drivetrain.alignToReefAlgae())
        joystickRight.button(1).whileTrue(
//            Commands.sequence(
//                Manipulator.outtake().withTimeout(0.6),
//                Elevator.setTargetHeight(Position.Stowed)
//            )
            Manipulator.outtake()
        )

//        controller.a().whileTrue(Drivetrain.alignToTargetWithPIDController())

//        controller.b().onTrue(Commands.runOnce({
//            println("Setting desired target node to left branch.")
//            Drivetrain.currentTargetSelection = ReefBranchSide.Left
//        }))
//
//        controller.x().onTrue(Commands.runOnce({
//            println("Setting desired target node to right branch.")
//            Drivetrain.currentTargetSelection = ReefBranchSide.Right
//        }))

        // (The button with the yellow tape on it)
        joystickLeft.button(8).onTrue(Commands.runOnce({
            println("Zeroing gyro.")
            Drivetrain.zeroGyro()
        }).ignoringDisable(true))

        joystickLeft.button(16).onTrue(Commands.runOnce({
            println("Zeroing gyro.")
            Drivetrain.zeroGyro(offset = Rotation2d.fromDegrees(-60.0))
        }).ignoringDisable(true))



        // Left close middle
        joystickLeft.button(9)
            .and { Robot.isDisabled }
            .toggleOnTrue(Elevator.coast().ignoringDisable(true))

//        joystickLeft.button(14).onTrue(Elevator.runHoming())

        controller.a().onTrue(Elevator.setTargetHeight(Elevator.Position.Stowed))
        controller.b().onTrue(Elevator.setTargetHeight(Elevator.Position.MidBar))
        controller.x().onTrue(Elevator.setTargetHeight(Elevator.Position.LowBar))
        controller.y().onTrue(Elevator.setTargetHeight(Elevator.Position.HighBar))
        controller.pov(0).onTrue(Elevator.setTargetHeight(Elevator.Position.AlgaeMidBar))
        joystickLeft.button(2).onTrue(
            tossAlgae()
        )
//
        controller.leftBumper().whileTrue(Funnel.outtake())
        controller.rightBumper().onTrue(
            Commands.sequence(
                Commands.runOnce({
                    Manipulator.isIntakeRunning = true
                }),
                Commands.race(
                    Manipulator.intake(
                        driverFeedback = controller.rumble(
                            GenericHID.RumbleType.kBothRumble,
                            1.0,
                            0.5.seconds,
                        )
                    ),
                    Funnel.intake()
                )
            ).andThen({
                Manipulator.isIntakeRunning = false
            })
        )

        controller.rightTrigger().onTrue(
            Commands.runOnce({
                Manipulator.isIntakeRunning = false
            })
        )

        controller.leftTrigger().onTrue(
            Commands.sequence(
                Commands.runOnce({
                    Manipulator.isIntakeRunning = true
                }),
                Manipulator.intakeAlgae(),
            )
        )

        // TODO: find a good button for this if needed
//        joystickRight.button(2).whileTrue(
//            Commands.parallel(
//                Manipulator.intakeNoRaceWithOutInterrupt(),
//                Funnel.intake()
//            )
//        )
        joystickRight.button(2).whileTrue(Drivetrain.alignToBarge())
        joystickRight.button(14).toggleOnTrue(Drivetrain.calculateWheelRadius())

//            Manipulator.intake()


//        controller.leftBumper().onTrue(Commands.runOnce(SignalLogger::start))
//        controller.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop))
////
//        controller.y().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        controller.a().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//        controller.b().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        controller.x().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /** Add data to the driver station dashboard. */
    private fun configureDashboard() {
        Dashboard.showTeleopTab(Shuffleboard.getTab("Teleoperated"))
    }

    override fun disabledInit() {
        if (model == Model.SIMULATION) {
            SimulatedArena.getInstance().resetFieldForAuto()
        }
    }

    override fun simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic()

        Logger.recordOutput(
            "FieldSimulation/Algae",
            *SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
        )
        Logger.recordOutput(
            "FieldSimulation/Coral",
            *SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
        )

    }

    private fun reportDiagnostics() {
        Diagnostics.periodic()
        Diagnostics.report(rioCANBus)
        Diagnostics.report(canivore)
        Diagnostics.reportDSPeripheral(joystickLeft.hid, isController = false)
        Diagnostics.reportDSPeripheral(joystickRight.hid, isController = false)
        Diagnostics.reportDSPeripheral(controller.hid, isController = true)
    }

    override fun robotPeriodic() {
        Dashboard.update()
        reportDiagnostics()

        CommandScheduler.getInstance().run()

        Diagnostics.send()
    }

    override fun autonomousInit() {
        Drivetrain.zeroGyro(true)
        autoCommand = Dashboard.autoChooser.selected
        autoCommand?.schedule()
    }

    override fun teleopInit() {
        autoCommand?.cancel()
    }

    override fun testInit() {
    }

    override fun testExit() {
    }

    /** A model of robot, depending on where we're deployed to. */
    enum class Model {
        SIMULATION, COMPETITION, PROTOTYPE
    }

    /** The model of this robot. */
    val model: Model = if (isSimulation()) {
        Model.SIMULATION
    } else {
        when (val key = Preferences.getString("Model", "competition")) {
            "competition" -> Model.COMPETITION
            "prototype" -> Model.PROTOTYPE
            else -> throw AssertionError("Invalid model found in preferences: $key")
        }
    }
}
