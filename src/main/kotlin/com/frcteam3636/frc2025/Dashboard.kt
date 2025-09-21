package com.frcteam3636.frc2025

import com.frcteam3636.frc2025.subsystems.drivetrain.autos.OnePieceCoral
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.StartingPosition
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.ThreePieceCoral
import com.frcteam3636.frc2025.subsystems.drivetrain.autos.TwoPieceCoral
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.Logger

object Dashboard {
//    private val field = Field2d()
    val autos = listOf(
        OnePieceCoral(StartingPosition.Left),
        OnePieceCoral(StartingPosition.Right),
        TwoPieceCoral(StartingPosition.Left),
        TwoPieceCoral(StartingPosition.Right),
        ThreePieceCoral(StartingPosition.Left),
        ThreePieceCoral(StartingPosition.Right)
    )
    val autoChooser = SendableChooser<Command>().apply {
        setDefaultOption("None", Commands.none())
        autos.forEach { addOption(it.name, it.autoSequence()) }
    }
    val defaultAuto: Command? = Commands.none()


    fun initialize() {
        PathPlannerLogging.setLogTargetPoseCallback {
            Logger.recordOutput("/Drivetrain/Target Pose", it)
        }
        PathPlannerLogging.setLogActivePathCallback {
            Logger.recordOutput("/Drivetrain/Desired Path", *it.toTypedArray())
        }
        autoChooser.setDefaultOption("None", defaultAuto)
        for (auto in autos) {
            autoChooser.addOption(auto.name, auto.autoSequence())
        }
        SmartDashboard.putData(autoChooser)
    }
}
