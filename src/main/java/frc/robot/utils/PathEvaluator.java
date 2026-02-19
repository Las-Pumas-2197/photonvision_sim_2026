package frc.robot.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DetectedRobot;
import frc.robot.utils.TunnelPath.Corridor;

/**
 * Evaluates and scores crossing paths based on obstacles and robot position.
 * Used to select the best open path for cycling.
 */
public class PathEvaluator {

    // Scoring weights
    private static final double WEIGHT_CLEAR_PATH = 100.0;      // Bonus for clear path
    private static final double WEIGHT_DISTANCE = -1.0;         // Penalty per meter to entry
    private static final double WEIGHT_OBSTACLE = -500.0;       // Heavy penalty for obstacles in path

    // Abort thresholds
    private static final double ABORT_DISTANCE_THRESHOLD = 3.0; // Only abort if obstacle is within 3m

    private PathEvaluator() {}

    /**
     * Result of evaluating a single path.
     */
    public record PathScore(
        TunnelPath path,
        double score,
        boolean isBlocked,
        List<DetectedRobot> obstaclesInPath
    ) implements Comparable<PathScore> {

        @Override
        public int compareTo(PathScore other) {
            // Higher score is better
            return Double.compare(other.score, this.score);
        }
    }

    /**
     * Evaluates all four crossing paths and returns them sorted by score.
     *
     * @param robotPose Current robot pose
     * @param obstacles List of detected obstacles
     * @return List of PathScore sorted by score (best first)
     */
    public static List<PathScore> evaluateAllPaths(Pose2d robotPose, List<DetectedRobot> obstacles) {
        List<PathScore> scores = new ArrayList<>();

        for (TunnelPath path : TunnelPath.getAllPaths()) {
            scores.add(evaluatePath(path, robotPose, obstacles));
        }

        Collections.sort(scores);
        return scores;
    }

    /**
     * Evaluates a single path and calculates its score.
     *
     * @param path The path to evaluate
     * @param robotPose Current robot pose
     * @param obstacles List of detected obstacles
     * @return PathScore with calculated metrics
     */
    public static PathScore evaluatePath(TunnelPath path, Pose2d robotPose, List<DetectedRobot> obstacles) {
        Corridor corridor = path.corridor();

        // Find obstacles in this path's corridor
        List<DetectedRobot> obstaclesInPath = obstacles.stream()
            .filter(obs -> obs.isInCorridor(corridor))
            .toList();

        // Path is blocked if any obstacle is present
        boolean isBlocked = !obstaclesInPath.isEmpty();

        // Calculate distance to path entry
        double distanceToEntry = path.distanceToEntry(robotPose.getTranslation());

        // Calculate score
        double score = 0.0;

        // Bonus for clear path
        if (!isBlocked) {
            score += WEIGHT_CLEAR_PATH;
        }

        // Distance penalty (closer is better)
        score += distanceToEntry * WEIGHT_DISTANCE;

        // Obstacle penalty
        score += obstaclesInPath.size() * WEIGHT_OBSTACLE;

        return new PathScore(path, score, isBlocked, new ArrayList<>(obstaclesInPath));
    }

    /**
     * Selects the best available path that is not blocked.
     * Prioritizes closest open path.
     *
     * @param robotPose Current robot pose
     * @param obstacles List of detected obstacles
     * @return The best unblocked path
     */
    public static TunnelPath selectBestPath(Pose2d robotPose, List<DetectedRobot> obstacles) {
        List<PathScore> scores = evaluateAllPaths(robotPose, obstacles);

        // Return best unblocked path
        for (PathScore score : scores) {
            if (!score.isBlocked()) {
                return score.path();
            }
        }

        // Fallback: return the highest scored path even if blocked
        // (requirement states at least one path will always be free)
        return scores.get(0).path();
    }

    /**
     * Determines if the current path should be aborted due to a new obstacle.
     * Only triggers abort if obstacle is close and ahead of robot.
     *
     * @param currentPath The path currently being executed
     * @param robotPose Current robot pose
     * @param obstacles List of detected obstacles
     * @return true if path should be aborted and re-planned
     */
    public static boolean shouldAbortPath(TunnelPath currentPath, Pose2d robotPose, List<DetectedRobot> obstacles) {
        Corridor corridor = currentPath.corridor();

        // Find obstacles in current corridor
        List<DetectedRobot> obstaclesInCorridor = obstacles.stream()
            .filter(obs -> obs.isInCorridor(corridor))
            .toList();

        if (obstaclesInCorridor.isEmpty()) {
            return false;
        }

        // Check if any obstacle is close enough and ahead to warrant abort
        Translation2d robotPosition = robotPose.getTranslation();
        boolean isWestbound = currentPath.isWestbound(robotPosition.getX());

        for (DetectedRobot obstacle : obstaclesInCorridor) {
            double distance = robotPosition.getDistance(obstacle.position());

            if (distance < ABORT_DISTANCE_THRESHOLD) {
                // Check if obstacle is ahead of robot in travel direction
                boolean obstacleAhead = isWestbound
                    ? obstacle.position().getX() < robotPosition.getX()
                    : obstacle.position().getX() > robotPosition.getX();

                if (obstacleAhead) {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Gets a human-readable summary of path scores for debugging.
     *
     * @param scores List of evaluated path scores
     * @return Formatted string summarizing scores
     */
    public static String getScoreSummary(List<PathScore> scores) {
        StringBuilder sb = new StringBuilder();
        sb.append("Path Scores:\n");

        for (PathScore score : scores) {
            sb.append(String.format("  %s: %.1f %s (obstacles: %d)\n",
                score.path().name(),
                score.score(),
                score.isBlocked() ? "[BLOCKED]" : "[OPEN]",
                score.obstaclesInPath().size()
            ));
        }

        return sb.toString();
    }
}
