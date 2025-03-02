package com.frcteam3636.frc2025.utils.swerve

import com.frcteam3636.frc2025.utils.math.inMetersPerSecond
import com.frcteam3636.frc2025.utils.math.metersPerSecond
import com.frcteam3636.frc2025.utils.math.radiansPerSecond
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity

enum class DrivetrainCorner {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
}

data class PerCorner<T>(val frontLeft: T, val frontRight: T, val backLeft: T, val backRight: T) :
    Collection<T> {
    operator fun get(corner: DrivetrainCorner): T =
        when (corner) {
            DrivetrainCorner.FRONT_LEFT -> frontLeft
            DrivetrainCorner.BACK_LEFT -> backLeft
            DrivetrainCorner.BACK_RIGHT -> backRight
            DrivetrainCorner.FRONT_RIGHT -> frontRight
        }

    fun <U> map(block: (T) -> U): PerCorner<U> = mapWithCorner { x, _ -> block(x) }
    fun <U> mapWithCorner(block: (T, DrivetrainCorner) -> U): PerCorner<U> = generate { corner ->
        block(this[corner], corner)
    }

    fun <U> zip(second: PerCorner<U>): PerCorner<Pair<T, U>> = generate { corner ->
        Pair(this[corner], second[corner])
    }

    private fun sequence(): Sequence<T> = sequenceOf(frontLeft, frontRight, backLeft, backRight)
    override fun iterator(): Iterator<T> = sequence().iterator()

    override val size: Int = 4

    override fun isEmpty(): Boolean = false

    override fun containsAll(elements: Collection<T>): Boolean = elements.all { contains(it) }

    override fun contains(element: T): Boolean = sequence().contains(element)

    companion object {
        fun <T> generate(block: (DrivetrainCorner) -> T): PerCorner<T> =
            PerCorner(
                frontLeft = block(DrivetrainCorner.FRONT_LEFT),
                frontRight = block(DrivetrainCorner.FRONT_RIGHT),
                backLeft = block(DrivetrainCorner.BACK_LEFT),
                backRight = block(DrivetrainCorner.BACK_RIGHT),
            )

        fun <T> fromConventionalArray(array: Array<T>): PerCorner<T> =
            PerCorner(
                frontLeft = array[0],
                frontRight = array[1],
                backLeft = array[2],
                backRight = array[3],
            )
    }
}

fun SwerveDriveKinematics.toCornerSwerveModuleStates(
    speeds: ChassisSpeeds
): PerCorner<SwerveModuleState> = PerCorner.fromConventionalArray(toSwerveModuleStates(speeds))

fun SwerveDriveKinematics.cornerStatesToChassisSpeeds(
    states: PerCorner<SwerveModuleState>
): ChassisSpeeds = toChassisSpeeds(*states.toList().toTypedArray())

fun SwerveDriveKinematics(translations: PerCorner<Translation2d>) =
    SwerveDriveKinematics(*translations.toList().toTypedArray())

/** The speed of the swerve module. */
var SwerveModuleState.speed: LinearVelocity
    get() = speedMetersPerSecond.metersPerSecond
    set(value) {
        speedMetersPerSecond = value.inMetersPerSecond()
    }

/** This swerve module state as a `Translation2d`. */
val SwerveModuleState.translation2dPerSecond: Translation2d
    get() = Translation2d(speedMetersPerSecond, angle)

val ChassisSpeeds.translation2dPerSecond: Translation2d
    get() = Translation2d(vxMetersPerSecond, vyMetersPerSecond)

val ChassisSpeeds.angularVelocity: AngularVelocity
    get() = omegaRadiansPerSecond.radiansPerSecond
