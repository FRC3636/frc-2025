package com.frcteam3636.frc2025

import com.frcteam3636.frc2025.subsystems.drivetrain.autos.AutoMode
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.OnePieceLeft
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.TwoPieceLeft
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.Logger

object Dashboard {
//    private val field = Field2d()
    val autoChooser = SendableChooser<Command>()

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
        autoChooser.setDefaultOption("None", Commands.none())
        autoChooser.addOption(OnePieceLeft.name, OnePieceLeft.autoSequence())
        autoChooser.addOption(TwoPieceLeft.name, TwoPieceLeft.autoSequence())
        SmartDashboard.putData(autoChooser)
    }
}
