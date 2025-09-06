package com.frcteam3636.frc2025

import com.ctre.phoenix6.CANBus
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.Gyro
import com.frcteam3636.frc2025.utils.cachedStatus
import com.frcteam3636.frc2025.utils.math.hasElapsed
import com.frcteam3636.frc2025.utils.math.seconds
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.InstantCommand
import java.net.InetAddress
import kotlin.concurrent.thread

/**
 * Reports diagnostics and sends notifications to the driver station.
 *
 * Each diagnostic condition is stored as a boolean value, and alerts are generated when one
 * becomes problematic. The alerts are sent to the driver dashboard and logged to the console.
 */
object Diagnostics {
    val timer = Timer()

    sealed class Fault(message: String, alertType: AlertType = AlertType.kError) {
        val alert = Alert(message, alertType)

        object GyroDisconnected : Fault("Failed to connect to gyro, vision and odometry will likely not function.")
        object LimelightDisconnected : Fault("Failed to connect to one or more LimeLights, vision will be impaired.")
        object DubiousAutoChoice :
            Fault(
                "There is no auto selected. Are you absolutely sure you **do not** want to run an auto?",
                AlertType.kWarning
            )

        object JoystickDisconnected :
            Fault("One or more Joysticks have disconnected, driver controls will not work.")

        object ControllerDisconnected :
            Fault("An Xbox Controller has disconnected, operator controls will not work.")

        object HIDDeviceIsWrongType :
            Fault(
                "Check USB device order in Driver Station! The connected devices are likely in the wrong order.",
                AlertType.kWarning
            )

        class CAN private constructor(bus: CANBus) {
            private class BusFailure(bus: CANBus) : Fault("The \"${bus.humanReadableName}\" CAN bus has FAILED!")
            private class BusError(bus: CANBus) :
                Fault("Devices on the \"${bus.humanReadableName}\" CAN bus are experiencing errors.")

            val failure: Fault = BusFailure(bus)
            val error: Fault = BusError(bus)

            companion object {
                private val knownBuses = HashMap<CANBus, CAN>()
                fun bus(bus: CANBus): CAN = knownBuses.getOrPut(bus) { CAN(bus) }
            }
        }
    }

    private var faults = HashSet<Fault>()

    fun reset() {
        synchronized(faults) {
            faults.clear()
        }
        timer.reset()
    }

    fun reportFault(fault: Fault) {
        synchronized(faults) {
            faults += fault
        }
    }

    private val errorResetTimer = Timer().apply { start() }
    private val knownCANBusErrors = HashMap<String, Int>()

    /** Report the CAN Bus's errors */
    fun report(canBus: CANBus) {
        val status = canBus.cachedStatus

        // Can't connect to the CAN Bus at all? It's probably unplugged or might have even failed.
        if (status.Status.isError) {
            reportFault(Fault.CAN.bus(canBus).failure)
            return
        }

        // If there are errors, the wiring probably disconnected or a motor isn't working.
        val knownErrors = knownCANBusErrors[canBus.name] ?: 0
        if (status.REC + status.TEC > knownErrors) {
            reportFault(Fault.CAN.bus(canBus).error)
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
            reportFault(Fault.GyroDisconnected)
        }
    }

    fun reportDSPeripheral(controller: GenericHID, isController: Boolean) {
        if (!controller.isConnected) {
            if (isController) {
                reportFault(Fault.ControllerDisconnected)
            } else {
                reportFault(Fault.JoystickDisconnected)
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
            reportFault(Fault.HIDDeviceIsWrongType)
        }
    }

    private val limelightsSync = Any()
    private var limelightsConnected = false
    fun reportLimelightsInBackground(names: Array<String>) {
        thread {
            while (true) {
                val allReachable = names.asIterable().all { name ->
                    try {
                        InetAddress.getByName("$name.local").isReachable(1000)
                    } catch (_: Exception) {
                        false
                    }
                }

                synchronized(limelightsSync) {
                    limelightsConnected = allReachable
                }

                Thread.sleep(5_000)
            }
        }
    }

    fun periodic() {
        reset()

        val selectedAuto = Dashboard.autoChooser.selected
        if (selectedAuto is InstantCommand) {
            reportFault(Fault.DubiousAutoChoice)
        }

        if (!Drivetrain.limelightsConnected)
            reportFault(Fault.LimelightDisconnected)
    }

    private var previousFaults = HashSet<Fault>()

    /** Show pending faults. */
    fun send() {
        for (fault in previousFaults) {
            fault.alert.set(false)
        }
        previousFaults.clear()

        synchronized(faults) {
            for (fault in faults) {
                fault.alert.set(true)
            }

            previousFaults.addAll(faults)
        }
    }
}

val CANBus.humanReadableName: String
    get() = if (name == "*") {
        "Canivore"
    } else {
        name
    }
