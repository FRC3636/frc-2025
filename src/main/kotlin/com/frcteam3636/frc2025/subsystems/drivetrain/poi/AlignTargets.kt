package com.frcteam3636.frc2025.subsystems.drivetrain.poi

import com.frcteam3636.frc2025.subsystems.drivetrain.APRIL_TAGS
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2025.utils.math.inches
import com.frcteam3636.frc2025.utils.math.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import kotlin.jvm.optionals.getOrNull

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
        val aprilTagPose = APRIL_TAGS
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

@Suppress("unused")
fun Iterable<TargetGroup>.closestTargetTo(pose: Pose2d): TargetSelection =
    flatMap { group ->
        group.targets.indices.map {
            TargetSelection(group, idx = it)
        }
    }.minByOrNull {
        it.pose.relativeTo(pose).translation.norm
    } ?: error("Can't find closest target")

fun Iterable<TargetGroup>.closestTargetToWithSelection(pose: Pose2d, reefBranchSide: ReefBranchSide): TargetSelection =
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
