package com.frcteam3636.frc2025.subsystems.drivetrain.poi

import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.subsystems.drivetrain.FIELD_LAYOUT
import com.frcteam3636.frc2025.utils.math.dot
import com.frcteam3636.frc2025.utils.math.inches
import com.frcteam3636.frc2025.utils.math.meters
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull
import kotlin.math.max
import kotlin.math.min

enum class ReefBranchSide {
    Left,
    Right,
}

interface AlignableTarget {
    val pose: Pose2d
}

data class TargetGroup(val targets: Array<AprilTagTarget>)

data class TargetSelection(
    val group: TargetGroup,
    val idx: Int = 0,
) {
    val pose: Pose2d
        get() = group.targets[idx].pose
}

/**
 * An alignment target relative to an April Tag's location.
 *
 * @param aprilTagId the ID of the tag the target should be placed relative to
 * @param offset An offset from the April Tag. The size of the robot is automatically added
 *               to this value so that it doesn't crash into
 */
class AprilTagTarget(aprilTagId: Int, offset: Translation2d) : AlignableTarget {
    override val pose: Pose2d

    constructor(aprilTagId: Int, side: ReefBranchSide) : this(
        aprilTagId,
        Translation2d(
            0.meters,
            // Move left/right from the april tag to get in front of the reef branch
            when (side) {
                ReefBranchSide.Left -> APRIL_TAG_HORIZONTAL_OFFSET + 0.5.inches
                ReefBranchSide.Right -> -APRIL_TAG_HORIZONTAL_OFFSET - 0.5.inches
            }
        ),
    )

    init {
        val aprilTagPose = FIELD_LAYOUT
            .getTagPose(aprilTagId)
            .orElseThrow {
                IllegalArgumentException(
                    "Can't make an April tag target for tag $aprilTagId because there's no tag with that ID"
                )
            }
            .toPose2d()

        val poseFacingAprilTag = aprilTagPose + Transform2d(
            Translation2d(),
            Rotation2d.k180deg,
        )

        val offsetFromPoseFacingAprilTagWithBumperSpacer = Translation2d(
            // We don't want to be *on top* of the april tag, so back up a bit from the tag.
            -Drivetrain.Constants.BUMPER_LENGTH / 2.0,
            0.meters,
        )
            .plus(offset)

        pose = poseFacingAprilTag + Transform2d(offsetFromPoseFacingAprilTagWithBumperSpacer, Rotation2d.kZero)
    }

    companion object {
        private fun branchTargetsFromIds(ids: IntRange): Array<TargetGroup> {
            return ids
                .map { id ->
                    TargetGroup(
                        arrayOf(
                            AprilTagTarget(id, ReefBranchSide.Left),
                            AprilTagTarget(id, ReefBranchSide.Right),
                        )
                    )
                }
                .toTypedArray()
        }

        val redAllianceTargets: Array<TargetGroup> = arrayOf(
            // Reef branches
            *branchTargetsFromIds(6..11),
            // Processor
//            TargetGroup(arrayOf(AprilTagTarget(3, Translation2d()))),
            // Human Player Stations
//            TargetGroup(arrayOf(AprilTagTarget(1, Translation2d()))),
//            TargetGroup(arrayOf(AprilTagTarget(2, Translation2d())))
        )

        val blueAllianceTargets: Array<TargetGroup> = arrayOf(
            // Reef branches
            *branchTargetsFromIds(17..22),
            // Processor
//            TargetGroup(arrayOf(AprilTagTarget(16, Translation2d()))),
            // Human Player Stations
//            TargetGroup(arrayOf(AprilTagTarget(13, Translation2d()))),
//            TargetGroup(arrayOf(AprilTagTarget(12, Translation2d())))
        )

        val currentAllianceTargets: Array<TargetGroup>
            get() {
                return when (DriverStation.getAlliance().getOrNull()) {
                    DriverStation.Alliance.Red -> redAllianceTargets
                    else -> blueAllianceTargets
                }
            }
    }
}

val AprilTagFieldLayout.center: Translation2d
    get() {
        return Translation2d(
            (fieldLength / 2.0).meters,
            (fieldWidth / 2.0).meters,
        )
    }

data class BargeTargetZone(val start: Translation2d, val end: Translation2d) {
    val line: Translation2d get() = end - start
    val length get() = line.norm

    fun rotateAround(center: Translation2d, rotation: Rotation2d): BargeTargetZone {
        return BargeTargetZone(
            start.rotateAround(center, rotation),
            end.rotateAround(center, rotation),
        )
    }

    fun pointClosestTo(other: Translation2d): Translation2d {
        // mostly based on https://stackoverflow.com/a/28931906

        val line = line
        val length = length

        val lineScaledToUnitLength = line / length
        val otherToStart = other - start

        val projection = otherToStart.dot(lineScaledToUnitLength)
        return start + lineScaledToUnitLength * min(length, max(0.0, projection))
    }

    fun log(name: String) {
        Logger.recordOutput("$name/Start", Pose2d(start, Rotation2d()))
        Logger.recordOutput("$name/End", Pose2d(end, Rotation2d()))
    }

    companion object {
        val RED = BargeTargetZone(
            Translation2d(10.2.meters, 0.7.meters),
            Translation2d(10.2.meters, 3.18.meters),
        )
        val BLUE = RED.rotateAround(FIELD_LAYOUT.center, Rotation2d.k180deg)

        fun forAlliance(alliance: DriverStation.Alliance): BargeTargetZone {
            return when (alliance) {
                DriverStation.Alliance.Red -> RED
                DriverStation.Alliance.Blue -> BLUE
            }
        }
    }
}

class BargeTarget private constructor(override val pose: Pose2d) : AlignableTarget {

    companion object {
        fun closestTo(robot: Translation2d, alliance: DriverStation.Alliance): BargeTarget {
            val zone = BargeTargetZone.forAlliance(alliance)
            val closestPoint = zone.pointClosestTo(robot)
            val rotation = when (alliance) {
                DriverStation.Alliance.Red -> Rotation2d.k180deg
                DriverStation.Alliance.Blue -> Rotation2d.kZero
            }

            return BargeTarget(Pose2d(closestPoint, rotation))
        }
    }
}

@Suppress("unused")
fun Iterable<TargetGroup>.closestTargetTo(pose: Pose2d): TargetSelection =
    flatMap { group ->
        group.targets.indices.map {
            TargetSelection(group, idx = it)
        }
    }.minByOrNull {
        it.pose.relativeTo(pose).translation.norm
    } ?: error("Can't find closest target")

fun Iterable<TargetGroup>.closestTargetToPoseWithSelection(
    pose: Pose2d,
    reefBranchSide: ReefBranchSide
): TargetSelection =
    map { group ->
        if (group.targets.size >= reefBranchSide.ordinal + 1) {
            TargetSelection(group, idx = reefBranchSide.ordinal)
        } else {
            TargetSelection(group, idx = 0)
        }
    }.minByOrNull { it: TargetSelection ->
        it.pose.relativeTo(pose).translation.norm
    } ?: error("Can't find closest target")

private val APRIL_TAG_HORIZONTAL_OFFSET = 0.147525.meters
