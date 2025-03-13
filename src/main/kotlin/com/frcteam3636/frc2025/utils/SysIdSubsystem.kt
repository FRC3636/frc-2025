package com.frcteam3636.frc2025.utils

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

interface SysIdSubsystem {
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command
    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command
}
