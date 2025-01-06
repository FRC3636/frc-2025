package com.frcteam3636.frc2024.subsystems.drivetrain

import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PathPoint
import com.pathplanner.lib.pathfinding.LocalADStar
import com.pathplanner.lib.pathfinding.Pathfinder
import edu.wpi.first.math.Pair
import edu.wpi.first.math.geometry.Translation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

class LocalADStarAK : Pathfinder {
    private val io = ADStarIO()

    /**
     * Get if a new path has been calculated since the last time a path was retrieved
     *
     * @return True if a new path is available
     */
    override fun isNewPathAvailable(): Boolean {
        if (!Logger.hasReplaySource()) {
            io.updateIsNewPathAvailable()
        }
        Logger.processInputs("LocalADStarAK", io)
        return io.isNewPathAvailable
    }

    /**
     * Get the most recently calculated path
     *
     * @param constraints The path constraints to use when creating the path
     * @param goalEndState The goal end state to use when creating the path
     * @return The PathPlannerPath created from the points calculated by the pathfinder
     */
    override fun getCurrentPath(constraints: PathConstraints, goalEndState: GoalEndState): PathPlannerPath? {
        if (!Logger.hasReplaySource()) {
            io.updateCurrentPathPoints(constraints, goalEndState)
        }
        Logger.processInputs("LocalADStarAK", io)
        return if (io.currentPathPoints.isEmpty()) {
            null
        } else PathPlannerPath.fromPathPoints(io.currentPathPoints, constraints, goalEndState)
    }

    /**
     * Set the start position to pathfind from
     *
     * @param startPosition Start position on the field. If this is within an obstacle it will be
     * moved to the nearest non-obstacle node.
     */
    override fun setStartPosition(startPosition: Translation2d) {
        if (!Logger.hasReplaySource()) {
            io.adStar.setStartPosition(startPosition)
        }
    }

    /**
     * Set the goal position to pathfind to
     *
     * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
     * to the nearest non-obstacle node.
     */
    override fun setGoalPosition(goalPosition: Translation2d) {
        if (!Logger.hasReplaySource()) {
            io.adStar.setGoalPosition(goalPosition)
        }
    }

    /**
     * Set the dynamic obstacles that should be avoided while pathfinding.
     *
     * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
     * opposite corners of a bounding box.
     * @param currentRobotPos The current position of the robot. This is needed to change the start
     * position of the path to properly avoid obstacles
     */
    override fun setDynamicObstacles(
        obs: List<Pair<Translation2d, Translation2d>>, currentRobotPos: Translation2d
    ) {
        if (!Logger.hasReplaySource()) {
            io.adStar.setDynamicObstacles(obs, currentRobotPos)
        }
    }

    private class ADStarIO : LoggableInputs {
        var adStar = LocalADStar()
        var isNewPathAvailable = false
        var currentPathPoints = emptyList<PathPoint>()
        override fun toLog(table: LogTable) {
            table.put("IsNewPathAvailable", isNewPathAvailable)
            val pointsLogged = DoubleArray(currentPathPoints.size * 2)
            var idx = 0
            for (point in currentPathPoints) {
                pointsLogged[idx] = point.position.x
                pointsLogged[idx + 1] = point.position.y
                idx += 2
            }
            table.put("CurrentPathPoints", pointsLogged)
        }

        override fun fromLog(table: LogTable) {
            isNewPathAvailable = table["IsNewPathAvailable", false]
            val pointsLogged = table["CurrentPathPoints", DoubleArray(0)]
            val pathPoints: MutableList<PathPoint> = ArrayList()
            var i = 0
            while (i < pointsLogged.size) {
                pathPoints.add(PathPoint(Translation2d(pointsLogged[i], pointsLogged[i + 1]), null))
                i += 2
            }
            currentPathPoints = pathPoints
        }

        fun updateIsNewPathAvailable() {
            isNewPathAvailable = adStar.isNewPathAvailable
        }

        fun updateCurrentPathPoints(constraints: PathConstraints?, goalEndState: GoalEndState?) {
            val currentPath = adStar.getCurrentPath(constraints, goalEndState)
            currentPathPoints = if (currentPath != null) {
                currentPath.allPathPoints
            } else {
                emptyList()
            }
        }
    }
}
