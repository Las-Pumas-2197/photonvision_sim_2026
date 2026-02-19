package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.PathfindingConstants.k_basinCenter;

import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.DetectedRobot;
import frc.robot.subsystems.FrontLimelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.utils.PathEvaluator;
import frc.robot.utils.TunnelPath;

/**
 * Factory class for obstacle-aware teleop cycling commands.
 * Evaluates 4 crossing paths (top tunnel, top ramp, bottom ramp, bottom tunnel)
 * and selects the closest open one based on detected robot obstacles.
 */
public final class ObstacleAwareCycleCommands {

    private ObstacleAwareCycleCommands() {}

    // Turret shooting constants for lead correction (same as CycleCommands)
    private static final double TURRET_EXIT_VELOCITY = 8.0; // Average exit velocity (m/s)
    private static final double TURRET_LAUNCH_ANGLE = Math.toRadians(65); // Launch angle

    /**
     * Creates an obstacle-aware cycle command that:
     * 1. Evaluates all 4 crossing paths and selects the closest open one
     * 2. Pathfinds through the selected path to the opposite side
     * 3. Continuously monitors for obstacles during execution
     * 4. Aborts and re-plans if an obstacle appears in the current path
     * 5. Turret tracks the basin target throughout
     *
     * @param swerve The swerve subsystem
     * @param turret The turret subsystem
     * @param frontCamera The front Limelight subsystem for obstacle detection
     * @param cancelCondition Supplier that returns true when the command should cancel
     */
    public static Command createObstacleAwareCycleCommand(
            Swerve swerve,
            Turret turret,
            FrontLimelight frontCamera,
            BooleanSupplier cancelCondition) {

        // Use AtomicReference to allow mutation within lambda
        AtomicReference<TunnelPath> currentPathRef = new AtomicReference<>();
        AtomicReference<Boolean> wasAbortedRef = new AtomicReference<>(false);

        return defer(() -> {
            Pose2d currentPose = swerve.getPose();
            List<DetectedRobot> obstacles = frontCamera.getDetectedRobots();

            // Evaluate and select best path
            TunnelPath selectedPath = PathEvaluator.selectBestPath(currentPose, obstacles);
            currentPathRef.set(selectedPath);
            wasAbortedRef.set(false);

            // Log path selection
            SmartDashboard.putString("Cycle/SelectedPath", selectedPath.name());
            SmartDashboard.putString("Cycle/PathScores",
                PathEvaluator.getScoreSummary(PathEvaluator.evaluateAllPaths(currentPose, obstacles)));

            // Get exit pose based on which side of field robot is on
            Pose2d exitPose = selectedPath.getExitPose(currentPose.getX());

            // Create pathfinding command to cross to the other side
            Command pathfindSequence = swerve.pathfindToPose(exitPose, true, 1.0);

            // Create path monitor that checks for obstacles periodically
            Command pathMonitor = run(() -> {
                // Update obstacle positions
                frontCamera.updateDetectedRobots(swerve.getPose());

                // Check if we should abort
                if (PathEvaluator.shouldAbortPath(
                        currentPathRef.get(),
                        swerve.getPose(),
                        frontCamera.getDetectedRobots())) {

                    SmartDashboard.putBoolean("Cycle/Aborting", true);
                    wasAbortedRef.set(true);
                    pathfindSequence.cancel();
                }
            }).beforeStarting(() -> SmartDashboard.putBoolean("Cycle/Aborting", false));

            // Turret tracking command
            Command turretTrackingCommand = run(() -> {
                turret.aimAtFieldPoseWithLead(
                    swerve.getPose(),
                    k_basinCenter,
                    swerve.getFieldSpeeds(),
                    TURRET_EXIT_VELOCITY,
                    TURRET_LAUNCH_ANGLE
                );
            });

            // Combine into parallel command group
            Command cycleSequence = new ParallelDeadlineGroup(
                pathfindSequence,
                pathMonitor,
                turretTrackingCommand
            ).until(cancelCondition);

            // If aborted, recursively re-plan with new path
            return cycleSequence.andThen(
                either(
                    // If aborted, try again with re-evaluation
                    defer(() -> {
                        SmartDashboard.putString("Cycle/Status", "Re-planning...");
                        return createObstacleAwareCycleCommand(swerve, turret, frontCamera, cancelCondition);
                    }, Set.of(swerve, frontCamera)),
                    // If completed normally, we're done
                    runOnce(() -> SmartDashboard.putString("Cycle/Status", "Complete")),
                    // Condition: was aborted
                    wasAbortedRef::get
                )
            );
        }, Set.of(swerve, frontCamera));
    }

    /**
     * Creates a simpler version that only evaluates paths at the start (no continuous monitoring).
     * Useful for testing or when continuous re-evaluation isn't needed.
     *
     * @param swerve The swerve subsystem
     * @param turret The turret subsystem
     * @param frontCamera The front Limelight subsystem for obstacle detection
     * @param cancelCondition Supplier that returns true when the command should cancel
     */
    public static Command createSimpleObstacleAwareCycleCommand(
            Swerve swerve,
            Turret turret,
            FrontLimelight frontCamera,
            BooleanSupplier cancelCondition) {

        return defer(() -> {
            Pose2d currentPose = swerve.getPose();

            // Update camera with current pose
            frontCamera.updateDetectedRobots(currentPose);
            List<DetectedRobot> obstacles = frontCamera.getDetectedRobots();

            // Select best path
            TunnelPath selectedPath = PathEvaluator.selectBestPath(currentPose, obstacles);

            SmartDashboard.putString("Cycle/SelectedPath", selectedPath.name());

            // Get exit pose
            Pose2d exitPose = selectedPath.getExitPose(currentPose.getX());

            // Pathfinding command
            Command pathfindSequence = swerve.pathfindToPose(exitPose, true, 1.0);

            // Turret tracking
            Command turretTrackingCommand = run(() -> {
                turret.aimAtFieldPoseWithLead(
                    swerve.getPose(),
                    k_basinCenter,
                    swerve.getFieldSpeeds(),
                    TURRET_EXIT_VELOCITY,
                    TURRET_LAUNCH_ANGLE
                );
            });

            return new ParallelDeadlineGroup(pathfindSequence, turretTrackingCommand)
                .until(cancelCondition);
        }, Set.of(swerve, frontCamera));
    }

    /**
     * Creates a diagnostic command that displays path evaluation results
     * without actually executing any movement.
     *
     * @param swerve The swerve subsystem (for pose)
     * @param frontCamera The front Limelight subsystem
     */
    public static Command createPathDiagnosticCommand(Swerve swerve, FrontLimelight frontCamera) {
        return runOnce(() -> {
            Pose2d currentPose = swerve.getPose();
            frontCamera.updateDetectedRobots(currentPose);
            List<DetectedRobot> obstacles = frontCamera.getDetectedRobots();

            var scores = PathEvaluator.evaluateAllPaths(currentPose, obstacles);
            String summary = PathEvaluator.getScoreSummary(scores);

            System.out.println("=== Path Evaluation Diagnostic ===");
            System.out.println("Robot Position: " + currentPose);
            System.out.println("Obstacles Detected: " + obstacles.size());
            System.out.println(summary);
            System.out.println("Best Path: " + scores.get(0).path().name());
            System.out.println("================================");

            SmartDashboard.putString("Cycle/Diagnostic", summary);
        });
    }
}
