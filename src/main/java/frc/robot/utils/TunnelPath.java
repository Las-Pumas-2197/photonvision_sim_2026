package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Defines the four crossing paths and their corridor zones for obstacle-aware cycling.
 * Each path can be traversed in either direction (west-to-east or east-to-west).
 */
public record TunnelPath(
    String name,
    Corridor corridor,
    Pose2d westExitPose,    // Exit pose when traveling west (X decreasing)
    Pose2d eastExitPose     // Exit pose when traveling east (X increasing)
) {
    /**
     * Defines a rectangular corridor zone on the field.
     */
    public record Corridor(
        double minX,
        double maxX,
        double minY,
        double maxY,
        String name
    ) {
        /** Top tunnel corridor (Y ~ 6.5-7.5) */
        public static final Corridor TOP_TUNNEL = new Corridor(1.5, 8.0, 6.5, 7.5, "TOP_TUNNEL");

        /** Top ramp corridor (Y ~ 5-6) */
        public static final Corridor TOP_RAMP = new Corridor(1.5, 8.0, 5.0, 6.0, "TOP_RAMP");

        /** Bottom ramp corridor (Y ~ 2-3) */
        public static final Corridor BOTTOM_RAMP = new Corridor(1.5, 8.0, 2.0, 3.0, "BOTTOM_RAMP");

        /** Bottom tunnel corridor (Y ~ 0.5-1.5) */
        public static final Corridor BOTTOM_TUNNEL = new Corridor(1.5, 8.0, 0.5, 1.5, "BOTTOM_TUNNEL");

        /**
         * Checks if a point is within this corridor.
         *
         * @param point The point to check
         * @return true if point is inside the corridor bounds
         */
        public boolean contains(Translation2d point) {
            return point.getX() >= minX && point.getX() <= maxX
                && point.getY() >= minY && point.getY() <= maxY;
        }

        /**
         * Checks if a circular area overlaps with this corridor.
         *
         * @param center Center of the circle
         * @param radius Radius of the circle
         * @return true if circle overlaps with corridor
         */
        public boolean overlaps(Translation2d center, double radius) {
            return center.getX() >= minX - radius && center.getX() <= maxX + radius
                && center.getY() >= minY - radius && center.getY() <= maxY + radius;
        }

        /**
         * Gets the center point of this corridor.
         */
        public Translation2d getCenter() {
            return new Translation2d((minX + maxX) / 2, (minY + maxY) / 2);
        }
    }

    // The four crossing paths
    /** Top tunnel path (Y ~ 6.5-7.5) */
    public static final TunnelPath TOP_TUNNEL = new TunnelPath(
        "Top Tunnel",
        Corridor.TOP_TUNNEL,
        new Pose2d(1.5, 7.0, Rotation2d.fromDegrees(180)),   // West exit
        new Pose2d(8.0, 7.0, Rotation2d.fromDegrees(0))      // East exit
    );

    /** Top ramp path (Y ~ 5-6) */
    public static final TunnelPath TOP_RAMP = new TunnelPath(
        "Top Ramp",
        Corridor.TOP_RAMP,
        new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(180)),   // West exit
        new Pose2d(8.0, 5.5, Rotation2d.fromDegrees(0))      // East exit
    );

    /** Bottom ramp path (Y ~ 2-3) */
    public static final TunnelPath BOTTOM_RAMP = new TunnelPath(
        "Bottom Ramp",
        Corridor.BOTTOM_RAMP,
        new Pose2d(1.5, 2.5, Rotation2d.fromDegrees(180)),   // West exit
        new Pose2d(8.0, 2.5, Rotation2d.fromDegrees(0))      // East exit
    );

    /** Bottom tunnel path (Y ~ 0.5-1.5) */
    public static final TunnelPath BOTTOM_TUNNEL = new TunnelPath(
        "Bottom Tunnel",
        Corridor.BOTTOM_TUNNEL,
        new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(180)),   // West exit
        new Pose2d(8.0, 1.0, Rotation2d.fromDegrees(0))      // East exit
    );

    /** All four paths. */
    private static final List<TunnelPath> ALL_PATHS = List.of(
        TOP_TUNNEL,
        TOP_RAMP,
        BOTTOM_RAMP,
        BOTTOM_TUNNEL
    );

    /**
     * Gets all four available paths.
     */
    public static List<TunnelPath> getAllPaths() {
        return ALL_PATHS;
    }

    /**
     * Gets the appropriate exit pose based on the robot's current position.
     * If robot is on the west side (X < 4.5), exit to the east.
     * If robot is on the east side (X >= 4.5), exit to the west.
     *
     * @param robotX Current robot X position
     * @return The exit pose on the opposite side
     */
    public Pose2d getExitPose(double robotX) {
        if (robotX < 4.5) {
            return eastExitPose;  // Robot is west, exit east
        } else {
            return westExitPose;  // Robot is east, exit west
        }
    }

    /**
     * Gets the entry point for the path based on robot position.
     *
     * @param robotX Current robot X position
     * @return Entry position (center of corridor on robot's side)
     */
    public Translation2d getEntryPoint(double robotX) {
        double centerY = (corridor.minY() + corridor.maxY()) / 2;
        if (robotX < 4.5) {
            return new Translation2d(corridor.minX(), centerY);  // West entry
        } else {
            return new Translation2d(corridor.maxX(), centerY);  // East entry
        }
    }

    /**
     * Calculates the distance from a position to this path's entry point.
     *
     * @param from Starting position
     * @return Distance in meters
     */
    public double distanceToEntry(Translation2d from) {
        Translation2d entry = getEntryPoint(from.getX());
        return from.getDistance(entry);
    }

    /**
     * Checks if traveling this path means going west (X decreasing).
     *
     * @param robotX Current robot X position
     * @return true if direction is westward
     */
    public boolean isWestbound(double robotX) {
        return robotX >= 4.5;
    }
}
