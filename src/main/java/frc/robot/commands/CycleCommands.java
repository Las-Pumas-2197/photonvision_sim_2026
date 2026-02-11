package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.PathfindingConstants.k_basinCenter;
import static frc.robot.utils.Constants.VisionConstants.k_fieldlayout;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.simulation.SimulatedGameState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * Factory class for teleop cycling commands.
 */
public final class CycleCommands {

    private CycleCommands() {}

    // Tunnel tag pairs: each tunnel has a tag on each end
    // North tunnel: 22 <-> 23, South tunnel: 28 <-> 17
    private static final Map<Integer, Integer> TUNNEL_PAIRS = Map.of(
        22, 23,
        23, 22,
        28, 17,
        17, 28
    );

    // Final positions after exiting each tunnel tag
    private static final Map<Integer, Pose2d> TUNNEL_EXIT_POSITIONS = Map.of(
        17, new Pose2d(8, 1, Rotation2d.fromDegrees(90)),
        28, new Pose2d(1.5, 1, Rotation2d.fromDegrees(180)),
        22, new Pose2d(8, 7, Rotation2d.fromDegrees(270)),
        23, new Pose2d(1.5, 7, Rotation2d.fromDegrees(180))
    );

    /**
     * Creates a command that crosses through the closest tunnel to the other side.
     * Passes through the tunnel without stopping, then continues to final position.
     * Turret tracks targets throughout the pathfinding sequence.
     * Cancels if any manual drive input is detected.
     *
     * @param swerve The swerve subsystem
     * @param vision The vision subsystem
     * @param cancelCondition Supplier that returns true when the command should cancel
     */
    public static Command createCycleCommand(Swerve swerve, Vision vision, BooleanSupplier cancelCondition) {
        return defer(() -> {
            Pose2d currentPose = swerve.getPose();

            // Find the closest tunnel entrance (closest tag)
            int closestTag = -1;
            double closestDist = Double.MAX_VALUE;

            for (int tagId : TUNNEL_PAIRS.keySet()) {
                var tagPose = k_fieldlayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    double dist = currentPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
                    if (dist < closestDist) {
                        closestDist = dist;
                        closestTag = tagId;
                    }
                }
            }

            if (closestTag == -1) {
                return print("ERROR: Could not find any tunnel tags");
            }

            // Get the exit tag on the other side of this tunnel
            int exitTag = TUNNEL_PAIRS.get(closestTag);
            Pose2d exitWaypoint = getTagWaypoint(exitTag);
            Pose2d finalPosition = TUNNEL_EXIT_POSITIONS.get(exitTag);

            if (exitWaypoint == null || finalPosition == null) {
                return print("ERROR: Could not find exit tag " + exitTag);
            }

            // Cross through tunnel without stopping, then continue to final position
            Command pathfindSequence = sequence(
                swerve.pathfindToPose(exitWaypoint, true, 2.0),
                swerve.pathfindToPose(finalPosition, true)
            );

            // Turret tracking runs in parallel with pathfinding
            Command turretTrackingCommand = run(() -> {
                vision.getTurretCamera().aimAtFieldPose(swerve.getPose(), k_basinCenter);
            });

            // Cancel sequence if manual drive input detected
            return new ParallelDeadlineGroup(pathfindSequence, turretTrackingCommand)
                .until(cancelCondition);
        }, Set.of(swerve));
    }

    /**
     * Creates a command that rapidly fires 10 fuel balls from the turret.
     * Uses the turret's current yaw angle to aim and calculates exit velocity
     * based on distance to the hub target.
     *
     * @param gameState The simulated game state for shooting
     * @param turretYawSupplier Supplier for current turret yaw angle
     * @param shotCount Number of shots to fire
     * @param delayBetweenShots Delay between shots in seconds
     */
    public static Command createRapidFireCommand(
            SimulatedGameState gameState,
            DoubleSupplier turretYawSupplier,
            int shotCount,
            double delayBetweenShots) {

        Command[] shotCommands = new Command[shotCount];
        for (int i = 0; i < shotCount; i++) {
            shotCommands[i] = sequence(
                runOnce(() -> gameState.shootFuelFromTurret(turretYawSupplier.getAsDouble())),
                waitSeconds(delayBetweenShots)
            );
        }
        return sequence(shotCommands);
    }

    /**
     * Creates a rapid fire command with default settings (10 shots, 0.2s delay).
     */
    public static Command createRapidFireCommand(
            SimulatedGameState gameState,
            DoubleSupplier turretYawSupplier) {
        return createRapidFireCommand(gameState, turretYawSupplier, 10, 0.2);
    }

    /** Gets a waypoint pose at the tag's location with zero rotation. */
    private static Pose2d getTagWaypoint(int tagId) {
        var tagPose = k_fieldlayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return null;
        }
        return new Pose2d(
            tagPose.get().getX(),
            tagPose.get().getY(),
            new Rotation2d(0)
        );
    }
}
