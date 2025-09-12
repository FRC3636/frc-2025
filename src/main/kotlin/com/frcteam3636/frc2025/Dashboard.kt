package com.frcteam3636.frc2025

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.littletonrobotics.junction.Logger

object Dashboard {
//    private val field = Field2d()
    val autoChooser = AutoBuilder.buildAutoChooser()!!

    fun update() {
//        field.robotPose = Drivetrain.estimatedPose
    }

    fun initialize() {
        PathPlannerLogging.setLogTargetPoseCallback {
//            field.getObject("target pose").pose = it
            Logger.recordOutput("/Drivetrain/Target Pose", it)
        }
        PathPlannerLogging.setLogActivePathCallback {
//            field.getObject("path").poses = it
            Logger.recordOutput("/Drivetrain/Desired Path", *it.toTypedArray())
        }
        SmartDashboard.putData(autoChooser)
    }
}
