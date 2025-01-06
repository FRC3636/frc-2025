package com.frcteam3636.frc2024.utils

import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.core.JsonProcessingException
import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.PubSubOption
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.networktables.StringTopic

object Elastic {
    private val topic: StringTopic = NetworkTableInstance.getDefault()
        .getStringTopic("/Elastic/RobotNotifications")
    private val publisher: StringPublisher = topic.publish(
        PubSubOption.sendAll(true),
        PubSubOption.keepDuplicates(true)
    )
    private val objectMapper = ObjectMapper()

    /**
     * Sends an alert notification to the Elastic dashboard.
     * The alert is serialized as a JSON string before being published.
     *
     * @param alert the [ElasticNotification] object containing alert details
     */
    fun sendAlert(alert: ElasticNotification?) {
        try {
            publisher.set(objectMapper.writeValueAsString(alert))
        } catch (e: JsonProcessingException) {
            e.printStackTrace()
        }
    }
}

/**
 * Represents a notification object to be sent to the Elastic dashboard.
 * This object holds properties such as level, title, description, display time,
 * and dimensions
 * to control how the alert is displayed on the dashboard.
 */
data class ElasticNotification @JvmOverloads constructor(
    @field:JsonProperty("title") var title: String,
    @field:JsonProperty("description") var description: String,
    @field:JsonProperty("level") var level: NotificationLevel = NotificationLevel.ERROR,
    @field:JsonProperty("displayTime") var displayTimeMillis: Int = 10000,
    @field:JsonProperty("width") private val width: Double = 350.0,
    @field:JsonProperty("height") private val height: Double = -1.0
) {
    /**
     * Creates a new ElasticNotification with specified dimensions and default
     * display time.
     * If the height is below zero, it is automatically inferred based on screen
     * size.
     *
     * @param level       the level of the notification
     * @param title       the title text of the notification
     * @param description the descriptive text of the notification
     * @param width       the width of the notification display area
     * @param height      the height of the notification display area, inferred if
     * below zero
     */
    constructor(title: String, description: String, level: NotificationLevel, width: Double, height: Double) : this(
        title,
        description,
        level,
        10000,
        width,
        height
    )
}

/**
 * Represents the possible levels of notifications for the Elastic dashboard.
 * These levels are used to indicate the severity or type of notification.
 */
enum class NotificationLevel {
    INFO,  // Informational message
    WARNING,  // Warning message
    ERROR // Error message
}

/**
 * Widget names and properties:
 * https://frc-elastic.gitbook.io/docs/additional-features-and-references/widgets-list-and-properties-reference
 */
enum class ElasticWidgets(val widgetName: String) {
    // Some have spaces, some don't. It's inconsistent.
    SwerveDrive("SwerveDrive"),
    RadialGauge("Radial Gauge"),
    MatchTime("Match Time"),
}
