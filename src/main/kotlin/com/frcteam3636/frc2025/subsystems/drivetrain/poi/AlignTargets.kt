package com.frcteam3636.frc2025.subsystems.drivetrain.poi

import com.frcteam3636.frc2025.subsystems.drivetrain.APRIL_TAGS
import com.frcteam3636.frc2025.subsystems.drivetrain.Drivetrain
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull

enum class ReefBranchSide {
    Left,
    Right,
}

interface AlignableTarget {
    val pose: Pose2d
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
            Meters.zero(),
            // Move left/right from the april tag to get in front of the reef branch
            when (side) {
                ReefBranchSide.Right -> APRIL_TAG_HORIZONTAL_OFFSET
                ReefBranchSide.Left -> -APRIL_TAG_HORIZONTAL_OFFSET
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

        val offsetWithSpaceForBumpers = Translation2d(
            -Drivetrain.Constants.BUMPER_LENGTH / 1.5,
            // We don't want to be *on top* of the april tag, so back up a bit from the tag.
            Meters.zero()
        )
            .plus(offset)

        pose = poseFacingAprilTag + Transform2d(offsetWithSpaceForBumpers, Rotation2d.kZero)
    }

    companion object {
        /** Makes a list of reef side targets. The given april tags are used to determine the pose of the target. */
        private fun branchTargetsFromIds(ids: IntRange): Array<AprilTagTarget> {
            return ids
                .flatMap { id ->
                    ReefBranchSide.entries.map { side ->
                        AprilTagTarget(id, side)
                    }
                }
                .toTypedArray()
        }

        val redAllianceTargets = arrayOf(
            // Reef branches
            *branchTargetsFromIds(6..11),
            // Processor
            AprilTagTarget(3, Translation2d()),
            // Human Player Stations
            AprilTagTarget(1, Translation2d()),
            AprilTagTarget(2, Translation2d())
        )

        val blueAllianceTargets = arrayOf(
            // Reef branches
            *branchTargetsFromIds(17..22),
            // Processor
            AprilTagTarget(16, Translation2d()),
            // Human Player Stations
            AprilTagTarget(13, Translation2d()),
            AprilTagTarget(12, Translation2d())
        )

        val currentAllianceTargets: Array<AprilTagTarget>
            get() {
                return when (DriverStation.getAlliance().getOrNull()) {
                    DriverStation.Alliance.Red -> redAllianceTargets
                    else -> blueAllianceTargets
                }
            }
    }
}

/**
 * Returns the target closest to the given pose.
 */
fun Iterable<AprilTagTarget>.closestTargetTo(pose: Pose2d): AprilTagTarget =
    minByOrNull { it.pose.relativeTo(pose).translation.norm }
        ?: error("Can't find closest target ")


private val APRIL_TAG_HORIZONTAL_OFFSET = Meters.of(0.147525)
