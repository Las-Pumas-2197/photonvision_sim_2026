package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import static edu.wpi.first.units.Units.*;

/**
 * Manages simulated game state for the 2026 Rebuilt game.
 * Uses YAGSL's bundled maple-sim with fuel game pieces.
 *
 * NOTE: We override the SimulatedArena in RobotContainer BEFORE creating
 * subsystems to use the Arena2026Rebuilt with field obstacles.
 */
public class SimulatedGameState {

    // Hub positions from Arena2026Rebuilt (center of scoring zone)
    private static final Translation3d BLUE_HUB_TARGET = new Translation3d(4.5974, 4.034536, 1.5748);
    private static final Translation3d RED_HUB_TARGET = new Translation3d(11.938, 4.034536, 1.5748);

    // Turret shooting constants
    private static final double TURRET_HEIGHT = 0.3; // meters (turret exit height)
    private static final double TURRET_LAUNCH_ANGLE = Math.toRadians(65); // fixed launch angle
    private static final double GRAVITY = 9.81; // m/s²
    private static final double MIN_EXIT_VELOCITY = 3.0; // minimum velocity (m/s)
    private static final double MAX_EXIT_VELOCITY = 15.0; // maximum velocity (m/s)

    // Scoring tolerance (radius of scoring zone ~0.6m)
    private static final Translation3d HUB_TOLERANCE = new Translation3d(0.6, 0.6, 0.5);

    // Spawn position behind hub with drop height (x, y, height to drop from)
    private static final Translation3d BLUE_HUB_SPAWN = new Translation3d(5.0, 4.0, 2.0);
    private static final Translation3d RED_HUB_SPAWN = new Translation3d(12.5, 4.0, 2.0);

    // Score tracking
    private int fuelScored = 0;
    private int fuelShot = 0;

    // Track the spawn position for the most recent shot (used by static callback)
    private Translation3d lastSpawnPosition = BLUE_HUB_SPAWN;

    // Reference to get robot state
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    /**
     * Creates a new SimulatedGameState for the 2026 Rebuilt game.
     *
     * @param robotPoseSupplier Supplier for current robot pose
     * @param chassisSpeedsSupplier Supplier for current field-relative chassis speeds
     */
    public SimulatedGameState(
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    /**
     * Initializes the arena for autonomous mode.
     * Clears existing fuel and spawns new ones at starting positions.
     */
    public void resetFieldForAuto() {
        // Clear existing game pieces
        SimulatedArena.getInstance().clearGamePieces();

        // Reset scoring
        fuelScored = 0;
        fuelShot = 0;

        // Spawn fuel at starting positions across the field
        // Customize these positions for your game layout
        spawnFuel(new Translation2d(2.0, 2.0));
        spawnFuel(new Translation2d(4.0, 4.0));
        spawnFuel(new Translation2d(6.0, 2.0));
        spawnFuel(new Translation2d(6.0, 6.0));
        spawnFuel(new Translation2d(2.0, 6.0));
        spawnFuel(new Translation2d(8.0, 4.0));
        spawnFuel(new Translation2d(10.0, 2.0));
        spawnFuel(new Translation2d(10.0, 6.0));

        SmartDashboard.putNumber("Sim/FuelScored", fuelScored);
        SmartDashboard.putNumber("Sim/FuelShot", fuelShot);
    }

    /**
     * Spawns a fuel piece at the specified field position.
     *
     * @param position Field position to spawn the fuel
     */
    public void spawnFuel(Translation2d position) {
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(position));
    }

    /**
     * Spawns a fuel piece that drops from the specified 3D position.
     * Creates a projectile with minimal horizontal velocity that falls to the ground.
     *
     * @param position 3D position (x, y, height) to drop the fuel from
     */
    public void spawnDroppingFuel(Translation3d position) {
        RebuiltFuelOnFly dropping = new RebuiltFuelOnFly(
            new Translation2d(position.getX(), position.getY()),  // Field position
            new Translation2d(0, 0),                               // No offset
            new ChassisSpeeds(0, 0, 0),                           // No chassis movement
            new Rotation2d(0),                                     // Facing forward
            Meters.of(position.getZ()),                           // Drop height
            MetersPerSecond.of(0.1),                              // Minimal exit velocity
            Radians.of(0)                                          // Horizontal angle (will just drop)
        );

        SimulatedArena.getInstance().addGamePieceProjectile(dropping);
    }

    /**
     * Shoots a fuel piece from the robot toward the hub.
     *
     * @param shooterOffset Offset from robot center to shooter exit point
     * @param shooterHeight Height of the shooter exit (meters)
     * @param exitVelocity Fuel exit velocity (meters per second)
     * @param shooterAngle Shooter angle above horizontal (radians)
     */
    public void shootFuel(
            Translation2d shooterOffset,
            double shooterHeight,
            double exitVelocity,
            double shooterAngle) {

        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds speeds = chassisSpeedsSupplier.get();

        // Determine which hub to target based on robot position (shoot at closer hub)
        double distToBlue = robotPose.getTranslation().getDistance(
            new Translation2d(BLUE_HUB_TARGET.getX(), BLUE_HUB_TARGET.getY()));
        double distToRed = robotPose.getTranslation().getDistance(
            new Translation2d(RED_HUB_TARGET.getX(), RED_HUB_TARGET.getY()));
        boolean targetBlueHub = distToBlue < distToRed;
        Translation3d targetHub = targetBlueHub ? BLUE_HUB_TARGET : RED_HUB_TARGET;
        Translation3d spawnPosition = targetBlueHub ? BLUE_HUB_SPAWN : RED_HUB_SPAWN;

        RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            shooterOffset,
            speeds,
            robotPose.getRotation(),
            Meters.of(shooterHeight),
            MetersPerSecond.of(exitVelocity),
            Radians.of(shooterAngle)
        );

        // Store spawn position for this shot
        lastSpawnPosition = spawnPosition;

        // Configure target position, tolerance, and hit callback
        projectile
            .withTargetPosition(() -> targetHub)
            .withTargetTolerance(HUB_TOLERANCE)
            .withHitTargetCallBack(() -> {
                // Ball scored - this only fires on actual hits
                fuelScored++;
                SmartDashboard.putNumber("Sim/FuelScored", fuelScored);
                SmartDashboard.putString("Sim/LastShot", "SCORED!");
                // Spawn new fuel dropping from above behind the hub
                spawnDroppingFuel(spawnPosition);
            });

        // Configure trajectory callbacks for visualization only
        projectile.withProjectileTrajectoryDisplayCallBack(
            successPoses -> {
                logTrajectory(successPoses, "Sim/ScoredTrajectory");
            },
            missPoses -> {
                SmartDashboard.putString("Sim/LastShot", "Missed");
                logTrajectory(missPoses, "Sim/MissedTrajectory");
            }
        );

        SimulatedArena.getInstance().addGamePieceProjectile(projectile);
        fuelShot++;
        SmartDashboard.putNumber("Sim/FuelShot", fuelShot);
    }

    /**
     * Simplified shoot method with default parameters for typical shooter.
     */
    public void shootFuel() {
        shootFuel(
            new Translation2d(0.3, 0),  // Shooter 0.3m in front of robot center
            0.6,
            7.0,
            Math.toRadians(55)
        );
    }

    /**
     * Shoots fuel from the turret at the calculated velocity needed to hit the hub.
     * Uses the turret's yaw angle to determine shooting direction and calculates
     * the required exit velocity based on distance to the target hub.
     *
     * @param turretYaw The turret yaw angle in radians (relative to robot heading)
     */
    public void shootFuelFromTurret(double turretYaw) {
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds speeds = chassisSpeedsSupplier.get();

        // Determine which hub to target based on robot position
        double distToBlue = robotPose.getTranslation().getDistance(
            new Translation2d(BLUE_HUB_TARGET.getX(), BLUE_HUB_TARGET.getY()));
        double distToRed = robotPose.getTranslation().getDistance(
            new Translation2d(RED_HUB_TARGET.getX(), RED_HUB_TARGET.getY()));
        boolean targetBlueHub = distToBlue < distToRed;
        Translation3d targetHub = targetBlueHub ? BLUE_HUB_TARGET : RED_HUB_TARGET;
        Translation3d spawnPosition = targetBlueHub ? BLUE_HUB_SPAWN : RED_HUB_SPAWN;

        // Calculate horizontal distance to hub
        double horizontalDistance = robotPose.getTranslation().getDistance(
            new Translation2d(targetHub.getX(), targetHub.getY()));

        // Calculate required exit velocity
        double exitVelocity = calculateRequiredVelocity(
            horizontalDistance,
            targetHub.getZ(),
            TURRET_HEIGHT,
            TURRET_LAUNCH_ANGLE
        );

        // Clamp velocity to safe range
        exitVelocity = Math.max(MIN_EXIT_VELOCITY, Math.min(MAX_EXIT_VELOCITY, exitVelocity));

        // Calculate shooting direction: robot heading + turret yaw
        Rotation2d shootingDirection = robotPose.getRotation().plus(new Rotation2d(turretYaw));

        // Turret offset (small offset in shooting direction)
        Translation2d turretOffset = new Translation2d(0.1, shootingDirection);

        RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            turretOffset,
            speeds,
            shootingDirection,
            Meters.of(TURRET_HEIGHT),
            MetersPerSecond.of(exitVelocity),
            Radians.of(TURRET_LAUNCH_ANGLE)
        );

        // Store spawn position for this shot
        lastSpawnPosition = spawnPosition;

        // Configure target position, tolerance, and hit callback
        projectile
            .withTargetPosition(() -> targetHub)
            .withTargetTolerance(HUB_TOLERANCE)
            .withHitTargetCallBack(() -> {
                fuelScored++;
                SmartDashboard.putNumber("Sim/FuelScored", fuelScored);
                SmartDashboard.putString("Sim/LastShot", "SCORED!");
                spawnDroppingFuel(spawnPosition);
            });

        // Configure trajectory callbacks for visualization only
        projectile.withProjectileTrajectoryDisplayCallBack(
            successPoses -> {
                logTrajectory(successPoses, "Sim/ScoredTrajectory");
            },
            missPoses -> {
                SmartDashboard.putString("Sim/LastShot", "Missed");
                logTrajectory(missPoses, "Sim/MissedTrajectory");
            }
        );

        SimulatedArena.getInstance().addGamePieceProjectile(projectile);
        fuelShot++;
        SmartDashboard.putNumber("Sim/FuelShot", fuelShot);
        SmartDashboard.putNumber("Sim/LastExitVelocity", exitVelocity);
        SmartDashboard.putNumber("Sim/LastDistance", horizontalDistance);
    }

    /**
     * Calculates the required exit velocity to hit a target at the given distance and height.
     * Uses projectile motion equations with a fixed launch angle.
     *
     * @param horizontalDistance Horizontal distance to target (meters)
     * @param targetHeight Height of target (meters)
     * @param launchHeight Height of launch point (meters)
     * @param launchAngle Launch angle above horizontal (radians)
     * @return Required exit velocity (m/s), or MAX_EXIT_VELOCITY if impossible
     */
    private double calculateRequiredVelocity(
            double horizontalDistance,
            double targetHeight,
            double launchHeight,
            double launchAngle) {

        double deltaH = targetHeight - launchHeight;
        double tanAngle = Math.tan(launchAngle);
        double cosAngle = Math.cos(launchAngle);

        // From projectile motion: v² = g * d² / (2 * cos²(θ) * (d * tan(θ) - Δh))
        double denominator = 2 * cosAngle * cosAngle * (horizontalDistance * tanAngle - deltaH);

        // If denominator is <= 0, the target is unreachable with this angle
        if (denominator <= 0) {
            return MAX_EXIT_VELOCITY;
        }

        double velocitySquared = GRAVITY * horizontalDistance * horizontalDistance / denominator;
        return Math.sqrt(velocitySquared);
    }

    /**
     * Returns the hub target position (for external use in aiming).
     * Returns the closer hub based on robot position.
     */
    public Translation3d getTargetHub() {
        Pose2d robotPose = robotPoseSupplier.get();
        double distToBlue = robotPose.getTranslation().getDistance(
            new Translation2d(BLUE_HUB_TARGET.getX(), BLUE_HUB_TARGET.getY()));
        double distToRed = robotPose.getTranslation().getDistance(
            new Translation2d(RED_HUB_TARGET.getX(), RED_HUB_TARGET.getY()));
        return distToBlue < distToRed ? BLUE_HUB_TARGET : RED_HUB_TARGET;
    }

    /**
     * Gets all fuel positions on the field for visualization.
     *
     * @return Array of fuel poses
     */
    public Pose3d[] getFuelPoses() {
        return SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    }

    /**
     * Updates the simulation. Call this in robotPeriodic().
     */
    public void update() {
        // Log fuel count
        Pose3d[] fuel = getFuelPoses();
        SmartDashboard.putNumber("Sim/FuelOnField", fuel.length);
        SmartDashboard.putNumber("Sim/Accuracy", getAccuracy());
    }

    /**
     * Gets the number of fuel scored this match.
     */
    public int getFuelScored() {
        return fuelScored;
    }

    /**
     * Gets the number of fuel shot this match.
     */
    public int getFuelShot() {
        return fuelShot;
    }

    /**
     * Gets the shooting accuracy as a percentage.
     */
    public double getAccuracy() {
        if (fuelShot == 0) return 0.0;
        return (double) fuelScored / fuelShot * 100.0;
    }

    /**
     * Logs a trajectory to SmartDashboard for AdvantageScope visualization.
     *
     * @param poses List of Pose3d representing the trajectory points
     * @param key The SmartDashboard key to log under
     */
    private void logTrajectory(java.util.List<Pose3d> poses, String key) {
        // Convert to double array: x, y, z, qw, qx, qy, qz per pose
        double[] trajectoryData = new double[poses.size() * 7];
        for (int i = 0; i < poses.size(); i++) {
            Pose3d p = poses.get(i);
            trajectoryData[i * 7 + 0] = p.getX();
            trajectoryData[i * 7 + 1] = p.getY();
            trajectoryData[i * 7 + 2] = p.getZ();
            trajectoryData[i * 7 + 3] = p.getRotation().getQuaternion().getW();
            trajectoryData[i * 7 + 4] = p.getRotation().getQuaternion().getX();
            trajectoryData[i * 7 + 5] = p.getRotation().getQuaternion().getY();
            trajectoryData[i * 7 + 6] = p.getRotation().getQuaternion().getZ();
        }
        SmartDashboard.putNumberArray(key, trajectoryData);
        SmartDashboard.putNumber(key + "Count", poses.size());
    }
}
