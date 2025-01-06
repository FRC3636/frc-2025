package com.frcteam3636.frc2024

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.utils.Elastic
import com.frcteam3636.frc2024.utils.ElasticNotification
import edu.wpi.first.hal.can.CANStatus
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController

/**
 * Reports diagnostics and sends notifications to the driver station.
 *
 * Each diagnostic condition is stored as a boolean value, and alerts are generated when one
 * becomes problematic. The alerts are sent to the driver dashboard and logged to the console.
 */
data class Diagnostics(
    val batteryFull: Boolean,
    val navXConnected: Boolean,
    val camerasConnected: Boolean,
    val errorStatusCodes: Map<String, StatusCode>,
    val canStatus: CANStatus,
) {

    private val alertMessages: List<ElasticNotification>

    init {
        val messages = mutableListOf<ElasticNotification>()

        // Add alerts for each suspicious diagnostic condition
        if (!navXConnected) messages.add(ElasticNotification("NavX disconnected", "The naxX gyro has disconnected!"))
        if (!camerasConnected) messages.add(ElasticNotification("Camera disconnected", "A camera has disconnected!"))
        if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
            messages.add(ElasticNotification("roboRIO CAN Errors", "CAN bus errors detected!"))
        }
        messages.addAll(errorStatusCodes.map { (name, code) -> ElasticNotification(name, code.description) })

        alertMessages = messages
    }

    /** Show the alerts that have not yet been displayed. */
    fun reportAlerts() {
        // only send new alerts
        val alerts = alertMessages.filter { it !in latest.alertMessages }

        for (alert in alerts) {
            DriverStation.reportWarning("ALERT: ${alert.title} - ${alert.description}", false)
            Elastic.sendAlert(alert) // show the alert as a notification on the driver station
        }

        latest = this
    }

    companion object {
        /** The most recent diagnostics. */
        var latest = Diagnostics(
            batteryFull = true,
            navXConnected = true,
            camerasConnected = true,
            errorStatusCodes = emptyMap(),
            canStatus = CANStatus(),
        )
            private set

        private const val FULL_BATTERY_VOLTAGE = 12.3

        /** Collect diagnostics from the robot's subsystems. */
        fun collect(signals: Map<String, StatusSignal<*>>): Diagnostics {
            val errorCodes = signals
                .filter { (_, signal) -> signal.status.isError }
                .mapValues { (_, signal) -> signal.status }

            return Diagnostics(
                batteryFull = RobotController.getBatteryVoltage() >= FULL_BATTERY_VOLTAGE,
                navXConnected = Drivetrain.inputs.gyroConnected,
                camerasConnected = Drivetrain.allCamerasConnected,
                errorStatusCodes = errorCodes,
                canStatus = RobotController.getCANStatus(),
            )
        }
    }

}
