package com.frcteam3636.frc2024

import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.utils.ElasticWidgets
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.Field2d

object Dashboard {
    private val field = Field2d()
    val autoChooser = AutoBuilder.buildAutoChooser()!!

    fun update() {
        field.robotPose = Drivetrain.estimatedPose
    }

    fun showTeleopTab(tab: ShuffleboardTab) {
        PathPlannerLogging.setLogTargetPoseCallback {
            field.getObject("target pose").pose = it
        }
        PathPlannerLogging.setLogActivePathCallback {
            field.getObject("path").poses = it
        }

        tab.add("Swerve Drive", Drivetrain)!!
            .withWidget(ElasticWidgets.SwerveDrive.widgetName)
            .withSize(4, 4)
            .withPosition(6, 0)

//        tab.addNumber("Velocity, meters per second") {
//            Translation2d(
//                Drivetrain.measuredChassisSpeeds.vxMetersPerSecond,
//                Drivetrain.measuredChassisSpeeds.vyMetersPerSecond
//            ).norm
//        }
//            .withWidget(ElasticWidgets.RadialGauge.widgetName)
//            // add a little extra to the max to improve visual spacing
//            .withProperties(
//                mapOf(
//                    "min_value" to 0.0,
//                    "max_value" to Drivetrain.Constants.FREE_SPEED.`in`(MetersPerSecond) + 4.0
//                )
//            )
//            .withSize(3, 3)
//            .withPosition(0, 0)

        tab.addNumber("Match Time") { DriverStation.getMatchTime() }
            .withWidget(ElasticWidgets.MatchTime.widgetName)
            .withSize(6, 4)
            .withPosition(0, 4)

        tab.add("Field", field)
            .withWidget(BuiltInWidgets.kField)
            .withSize(12, 6)
            .withPosition(12, 0)

        tab.add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(4, 2)
            .withPosition(10, 6)

        // Status Indicators
        tab.addNumber("Battery Voltage") { RobotController.getBatteryVoltage() }
            .withWidget(BuiltInWidgets.kVoltageView)
            .withSize(6, 2)
            .withPosition(0, 2)

        tab.addBoolean("NavX OK") { Diagnostics.latest.navXConnected }
            .withPosition(10, 0)
            .withSize(2, 1)
        tab.addBoolean("Cameras OK") { Diagnostics.latest.navXConnected }
            .withPosition(10, 1)
            .withSize(2, 1)
        tab.addBoolean("CAN Bus OK") { Diagnostics.latest.canStatus.transmitErrorCount == 0 && Diagnostics.latest.canStatus.receiveErrorCount == 0 }
            .withPosition(10, 2)
            .withSize(2, 1)
        tab.addBoolean("TalonFX OK") { Diagnostics.latest.errorStatusCodes.isEmpty() }
            .withPosition(10, 3)
            .withSize(2, 1)
        tab.addBoolean("Battery Full") { RobotController.getBatteryVoltage() >= 12.3 }
            .withPosition(10, 4)
            .withSize(2, 2)
        tab.addNumber("CAN Bus Utilization") { Diagnostics.latest.canStatus.percentBusUtilization * 100.0 }
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(mapOf("min_value" to 0.0, "max_value" to 100.0))
            .withPosition(14, 6)
            .withSize(4, 2)
    }
}
