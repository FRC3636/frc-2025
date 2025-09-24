package com.frcteam3636.frc2025

import com.ctre.phoenix6.CANBus
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.Gyro
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.StartingPosition
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.determineStartingPosition
import com.frcteam3636.frc2025.utils.cachedStatus
import com.frcteam3636.frc2025.utils.math.hasElapsed
import com.frcteam3636.frc2025.utils.math.seconds
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj.Timer
import java.net.InetAddress
import kotlin.concurrent.thread

/**
 * Reports diagnostics and sends notifications to the driver station.
 *
 * Each diagnostic condition is stored as a boolean value, and alerts are generated when one
 * becomes problematic or info should be provided.
 * The alerts are sent to the driver dashboard and logged to the console.
 */
object Diagnostics {
    val timer = Timer()

    sealed class RobotAlert(message: String, alertType: AlertType = AlertType.kError) {
        val alert = Alert(message, alertType)

        object GyroDisconnected : RobotAlert("Failed to connect to gyro, vision and odometry will likely not function.")
        object LimelightDisconnected : RobotAlert("Failed to connect to one or more LimeLights, vision will be impaired.")
        object DubiousAutoChoice :
            RobotAlert(
                "There is no auto selected. Are you absolutely sure you **do not** want to run an auto?",
                AlertType.kWarning
            )
        object NoAutoTags : RobotAlert("There are no visible Apriltags. Auto will assume a starting position based on older vision data (if available), please ensure Apriltag visibility to have accurate auto routines.", alertType = AlertType.kWarning)
        object GyroNotZeroedManually : RobotAlert("The gyro has not been zeroed manually. Gyro will be homed to the correct rotation automagically by vision <3.",
            AlertType.kInfo
        )
        object MegaTag1Active : RobotAlert("Megatag V1 is currently in use until the first enable. Please ensure that the robot knows it's rotation before enabling. If there is an Apriltag in view, this is fine.", AlertType.kInfo)
        object MegaTag2Active : RobotAlert("Megatag V2 is now in use for the remainder of this robot code run. If you are experiencing rotation issues, please zero the gyro manually using the button with the yellow tape.", AlertType.kInfo)
        object SelectedAutoLeft : RobotAlert("The robot has determined it is starting on the LEFT side. If this is wrong please ensure Apriltag visibility.", AlertType.kInfo)
        object SelectedAutoRight : RobotAlert("The robot has determined it is starting on the RIGHT side. If this is wrong please ensure Apriltag visibility.",
            AlertType.kInfo
        )

        object ThreadNotRealTime : RobotAlert("The main robot thread does not have real-time (RT) priority. Automatic functions may not work as expected and loop overruns may be present until this changes.",
            AlertType.kWarning
        )

        object JoystickDisconnected :
            RobotAlert("One or more Joysticks have disconnected, driver controls will not work.")

        object ControllerDisconnected :
            RobotAlert("An Xbox Controller has disconnected, operator controls will not work.")

        object HIDDeviceIsWrongType :
            RobotAlert(
                "Check USB device order in Driver Station! The connected devices are likely in the wrong order.",
                AlertType.kWarning
            )

        class CAN private constructor(bus: CANBus) {
            private class BusFailure(bus: CANBus) : RobotAlert("The \"${bus.humanReadableName}\" CAN bus has FAILED!")
            private class BusError(bus: CANBus) :
                RobotAlert("Devices on the \"${bus.humanReadableName}\" CAN bus are experiencing errors.")

            val failure: RobotAlert = BusFailure(bus)
            val error: RobotAlert = BusError(bus)

            companion object {
                private val knownBuses = HashMap<CANBus, CAN>()
                fun bus(bus: CANBus): CAN = knownBuses.getOrPut(bus) { CAN(bus) }
            }
        }
    }

    private var robotAlerts = HashSet<RobotAlert>()

    fun reset() {
        robotAlerts.clear()
        timer.reset()
    }

    fun reportAlert(robotAlert: RobotAlert) {
        robotAlerts += robotAlert
    }

    private val errorResetTimer = Timer().apply { start() }
    private val knownCANBusErrors = HashMap<String, Int>()

    /** Report the CAN Bus's errors */
    fun report(canBus: CANBus) {
        val status = canBus.cachedStatus

        // Can't connect to the CAN Bus at all? It's probably unplugged or might have even failed.
        if (status.Status.isError) {
            reportAlert(RobotAlert.CAN.bus(canBus).failure)
            return
        }

        // If there are errors, the wiring probably disconnected or a motor isn't working.
        val knownErrors = knownCANBusErrors[canBus.name] ?: 0
        if (status.REC + status.TEC > knownErrors) {
            reportAlert(RobotAlert.CAN.bus(canBus).error)
        }

        // Every second we record an "acceptable" number of errors so that if a
        // motor is plugged in after it has been erroring for a while, the alert will
        // dismiss itself.
        if (errorResetTimer.hasElapsed(5.seconds)) {
            knownCANBusErrors[canBus.name] = status.REC + status.TEC
        }
    }

    fun report(gyro: Gyro) {
        if (!gyro.connected) {
            reportAlert(RobotAlert.GyroDisconnected)
        }
    }

    fun reportDSPeripheral(controller: GenericHID, isController: Boolean) {
        if (!controller.isConnected) {
            if (isController) {
                reportAlert(RobotAlert.ControllerDisconnected)
            } else {
                reportAlert(RobotAlert.JoystickDisconnected)
            }
            return
        }

        val type = controller.type
        val isExpectedType = if (isController) {
            type == GenericHID.HIDType.kHIDGamepad || type == GenericHID.HIDType.kXInputGamepad
        } else {
            type == GenericHID.HIDType.kHIDJoystick || type == GenericHID.HIDType.kHIDFlight
        }

        if (!isExpectedType) {
            reportAlert(RobotAlert.HIDDeviceIsWrongType)
        }
    }

    fun periodic() {
        reset()

        // To save loop times, don't bother checking these if enabled
        if (Robot.isDisabled) {
            val selectedAuto = Dashboard.autoChooser.selected
            if (selectedAuto == AutoModes.None)
                reportAlert(RobotAlert.DubiousAutoChoice)
            if (!Robot.gyroOffsetManually && Robot.beforeFirstEnable)
                reportAlert(RobotAlert.GyroNotZeroedManually)
            if (!Drivetrain.tagsVisible)
                reportAlert(RobotAlert.NoAutoTags)
            if (determineStartingPosition() == StartingPosition.Left)
                reportAlert(RobotAlert.SelectedAutoLeft)
            else
                reportAlert(RobotAlert.SelectedAutoRight)
            if (Robot.beforeFirstEnable)
                reportAlert(RobotAlert.MegaTag1Active)
            else
                reportAlert(RobotAlert.MegaTag2Active)
        }

        if (!Drivetrain.limelightsConnected)
            reportAlert(RobotAlert.LimelightDisconnected)

        if (!Threads.getCurrentThreadIsRealTime())
            reportAlert(RobotAlert.ThreadNotRealTime)
    }

    private var previousRobotAlerts = HashSet<RobotAlert>()

    /** Show pending faults. */
    fun send() {
        for (fault in previousRobotAlerts) {
            fault.alert.set(false)
        }
        previousRobotAlerts.clear()

        for (fault in robotAlerts) {
            fault.alert.set(true)
        }

        previousRobotAlerts.addAll(robotAlerts)
    }
}

val CANBus.humanReadableName: String
    get() = if (name == "*") {
        "Canivore"
    } else {
        name
    }
