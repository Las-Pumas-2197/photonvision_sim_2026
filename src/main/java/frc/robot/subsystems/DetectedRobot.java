package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.TunnelPath;

/**
 * Immutable record representing a robot detected by the front Limelight camera.
 */
public record DetectedRobot(
    Translation2d position,     // Field-relative position
    double confidence,          // 0.0 to 1.0
    double timestamp            // FPGA timestamp when detected
) {
    /** Approximate robot radius (meters). */
    public static final double ROBOT_RADIUS = 0.5;

    /** Clearance margin around robots (meters). */
    public static final double CLEARANCE = 0.6;

    /** Total effective radius for collision avoidance. */
    public static final double EFFECTIVE_RADIUS = ROBOT_RADIUS + CLEARANCE;

    /**
     * Checks if this robot is within a path corridor, accounting for clearance.
     *
     * @param corridor The corridor to check
     * @return true if robot overlaps with corridor
     */
    public boolean isInCorridor(TunnelPath.Corridor corridor) {
        return position.getX() >= corridor.minX() - EFFECTIVE_RADIUS
            && position.getX() <= corridor.maxX() + EFFECTIVE_RADIUS
            && position.getY() >= corridor.minY() - EFFECTIVE_RADIUS
            && position.getY() <= corridor.maxY() + EFFECTIVE_RADIUS;
    }

    /**
     * Checks if this detection is stale (too old to be reliable).
     *
     * @param maxAge Maximum age in seconds
     * @return true if detection is older than maxAge
     */
    public boolean isStale(double maxAge) {
        return Timer.getFPGATimestamp() - timestamp > maxAge;
    }

    /**
     * Gets the effective radius including clearance margin.
     *
     * @return Total radius to avoid in meters
     */
    public double getEffectiveRadius() {
        return EFFECTIVE_RADIUS;
    }
}
