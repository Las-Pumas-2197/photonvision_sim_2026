package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.PathfindingConstants.*;
import static frc.robot.utils.Constants.SwerveDriveConstants.k_initpose;
import static frc.robot.utils.Constants.VisionConstants.k_fieldlayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * Factory class for autonomous command routines.
 */
public final class Autos {

    private Autos() {}

    /**
     * Auto that cycles through a tag waypoint to the field center and back.
     * Path: Start -> Under Tag -> Field Center -> Under Tag -> Start
     * Robot passes through waypoints continuously without stopping.
     */
    public static Command driveUnderTagAuto(Swerve swerve, int tagId) {
        var tagPose = k_fieldlayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return print("ERROR: Tag " + tagId + " not found in field layout");
        }

        Pose2d waypointPose = new Pose2d(
            tagPose.get().getX(),
            tagPose.get().getY(),
            new Rotation2d(0)
        );

        Pose2d fieldCenterPose = k_fieldCenter;
        final double waypointVelocity = 2.0;

        return sequence(
            defer(() -> swerve.pathfindToPose(
                new Pose2d(waypointPose.getX(), waypointPose.getY(), swerve.getPose().getRotation()),
                true,
                waypointVelocity
            ), java.util.Set.of(swerve)),
            defer(() -> swerve.pathfindToPose(
                new Pose2d(fieldCenterPose.getX(), fieldCenterPose.getY(), swerve.getPose().getRotation()),
                true,
                0.0
            ), java.util.Set.of(swerve)),
            defer(() -> swerve.pathfindToPose(
                new Pose2d(waypointPose.getX(), waypointPose.getY(), swerve.getPose().getRotation()),
                true,
                waypointVelocity
            ), java.util.Set.of(swerve)),
            defer(() -> swerve.pathfindToPose(
                new Pose2d(k_initpose.getX(), k_initpose.getY(), swerve.getPose().getRotation()),
                true,
                0.0
            ), java.util.Set.of(swerve))
        );
    }

    /** Simple drive test - no pathfinding, just drives forward for 2 seconds. */
    public static Command simpleDriveTest(Swerve swerve) {
        return run(() -> swerve.drive(new ChassisSpeeds(1.0, 0, 0)), swerve)
            .withTimeout(2.0);
    }

    /** Simple square pattern auto with turret tracking. */
    public static Command simpleSquareAuto(Swerve swerve, Vision vision) {
        SequentialCommandGroup driveSequence = new SequentialCommandGroup(
            swerve.pathfindToPose(k_squarePoint1, false),
            swerve.pathfindToPose(k_squarePoint2, false),
            swerve.pathfindToPose(k_squarePoint3, false),
            swerve.pathfindToPose(k_squarePoint4, false));

        Command turretTrackingCommand = run(() -> {
            vision.getTurretCamera().aimAtCenterOfTargets(swerve.getPose());
        });

        return new ParallelDeadlineGroup(driveSequence, turretTrackingCommand);
    }

    /** Autoalign sequence for the reef. */
    public static Command autoAlignReef(Swerve swerve, int fiducialID) {
        return new SequentialCommandGroup(
            swerve.pathfindToPose(k_bluereefA, true),
            swerve.pathfindToPose(k_redreefA, true));
    }
}
