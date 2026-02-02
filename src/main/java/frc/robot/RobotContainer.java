// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.BlackBox;
import frc.robot.utils.Telemetry;

import static frc.robot.utils.Constants.OIConstants.*;
import static frc.robot.utils.Constants.PathfindingConstants.*;
import static frc.robot.utils.Constants.SwerveDriveConstants.k_initpose;
import static frc.robot.utils.Constants.VisionConstants.k_fieldlayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

public class RobotContainer {

  // objects
  private final CommandXboxController m_joystick = new CommandXboxController(k_joystickport);
  private final Swerve m_swerve = new Swerve();
  private final Vision m_vision = new Vision();
  private final Telemetry m_telemetry = new Telemetry(m_vision, m_swerve);

  // autochooser
  private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

  public RobotContainer() {

    // set pathfinder type and configure path planner
    Pathfinding.setPathfinder(new LocalADStar());
    m_swerve.runAutoBuilder();

    // autos
    // m_autochooser.setDefaultOption("simple drive test", driveUnderTagAuto(28));
    m_autochooser.setDefaultOption("simple drive test", simpleSquareAuto());

    m_autochooser.addOption("square", simpleSquareAuto());
    m_autochooser.addOption("autoalign reef A", autoAlignReef(18));
    SmartDashboard.putData(m_autochooser);

    // default commands for subsystems
    m_swerve.setDefaultCommand(swerveDefaultCommand());
    m_vision.setDefaultCommand(visionDefaultCommand());

    // configure bindings
    configureBindings();
  }

  // triggers
  private void configureBindings() {
    m_joystick.back().onTrue(runOnce(() -> m_swerve.getCurrentCommand().cancel()));
    m_joystick.start().whileTrue(run(() -> BlackBox.DataRecorder.recordData("heading", m_swerve.getGyroHeading())));
    m_joystick.a().onTrue(createCycleCommand());
  }

  /**
   * Composed default command for photon vision subsystem. Do not interrupt or bad
   * things happen.
   */
  private ParallelCommandGroup visionDefaultCommand() {
    ParallelCommandGroup cmd = new ParallelCommandGroup();
    if (RobotBase.isSimulation())
      cmd.addCommands(run(() -> m_vision.updatePose(m_swerve.getSimPose())));
    cmd.addCommands(run(() -> m_swerve.addVisionMeasurements(m_vision.getEstimates())));
    cmd.addRequirements(m_vision);
    return cmd;
  }

  /** Composed default command for swerve subsystem with turret tracking. */
  private Command swerveDefaultCommand() {
    Command driveCommand = new RunCommand(
        () -> m_swerve.drive(new ChassisSpeeds(
            MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * k_maxlinspeedteleop,
            -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * k_maxlinspeedteleop,
            -MathUtil.applyDeadband(m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop)),
        m_swerve);

    Command turretTrackingCommand = run(() -> {
      m_vision.getTurretCamera().aimAtClosestTarget(m_swerve.getPose());
    });

    return new ParallelCommandGroup(driveCommand, turretTrackingCommand);
  }

  /** Autoalign sequence for the reef. */
  private SequentialCommandGroup autoAlignReef(int fiducialID) {
    return new SequentialCommandGroup(
        m_swerve.pathfindToPose(k_bluereefA, true),
        m_swerve.pathfindToPose(k_redreefA, true));
  }

  /**
   * Auto that cycles through a tag waypoint to the field center and back.
   * Path: Start -> Under Tag -> Field Center -> Under Tag -> Start
   * Robot passes through waypoints continuously without stopping.
   */
  private Command driveUnderTagAuto(int tagId) {
    // Get tag pose from field layout
    var tagPose = k_fieldlayout.getTagPose(tagId);
    if (tagPose.isEmpty()) {
      return print("ERROR: Tag " + tagId + " not found in field layout");
    }

    // Waypoint under the tag
    Pose2d waypointPose = new Pose2d(
        tagPose.get().getX(),
        tagPose.get().getY(),
        new Rotation2d(0)
    );

    // Field center pose
    Pose2d fieldCenterPose = k_fieldCenter;

    // Velocity to maintain through waypoints m/s
    final double waypointVelocity = 2.0;

    // Build the cycle sequence
    return sequence(
        defer(() -> m_swerve.pathfindToPose(
            new Pose2d(waypointPose.getX(), waypointPose.getY(), m_swerve.getPose().getRotation()),
            true,
            waypointVelocity
        ), java.util.Set.of(m_swerve)),
        defer(() -> m_swerve.pathfindToPose(
            new Pose2d(fieldCenterPose.getX(), fieldCenterPose.getY(), m_swerve.getPose().getRotation()),
            true,
            0.0 // stop at field center
        ), java.util.Set.of(m_swerve)),
        defer(() -> m_swerve.pathfindToPose(
            new Pose2d(waypointPose.getX(), waypointPose.getY(), m_swerve.getPose().getRotation()),
            true,
            waypointVelocity
        ), java.util.Set.of(m_swerve)),
        defer(() -> m_swerve.pathfindToPose(
            new Pose2d(k_initpose.getX(), k_initpose.getY(), m_swerve.getPose().getRotation()),
            true,
            0.0
        ), java.util.Set.of(m_swerve))
    );
  }

  /** Simple drive test - no pathfinding, just drives forward for 2 seconds. */
  private Command simpleDriveTest() {
    return run(() -> m_swerve.drive(new ChassisSpeeds(1.0, 0, 0)), m_swerve)
        .withTimeout(2.0);
  }

  // Tunnel tag pairs: each tunnel has a tag on each end
  // North tunnel: 22 <-> 23, South tunnel: 28 <-> 17
  private static final java.util.Map<Integer, Integer> TUNNEL_PAIRS = java.util.Map.of(
      22, 23,
      23, 22,
      28, 17,
      17, 28
  );

  // Final positions after exiting each tunnel tag
  private static final java.util.Map<Integer, Pose2d> TUNNEL_EXIT_POSITIONS = java.util.Map.of(
      17, new Pose2d(8, 1, Rotation2d.fromDegrees(90)),
      28, new Pose2d(1.5, 1, Rotation2d.fromDegrees(180)),
      22, new Pose2d(8, 7, Rotation2d.fromDegrees(270)),
      23, new Pose2d(1.5, 7, Rotation2d.fromDegrees(180))
  );

  /**
   * Creates a command that crosses through the closest tunnel to the other side.
   * Passes through the tunnel without stopping, then continues to final position.
   * Turret tracks targets throughout the pathfinding sequence.
   */
  private Command createCycleCommand() {
    return defer(() -> {
      Pose2d currentPose = m_swerve.getPose();

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
          m_swerve.pathfindToPose(exitWaypoint, true, 2.0),
          m_swerve.pathfindToPose(finalPosition, true)
      );

      // Turret tracking runs in parallel with pathfinding
      Command turretTrackingCommand = run(() -> {
        m_vision.getTurretCamera().aimAtClosestTarget(m_swerve.getPose());
      });

      return new ParallelDeadlineGroup(pathfindSequence, turretTrackingCommand);
    }, java.util.Set.of(m_swerve));
  }

  /** Gets a waypoint pose at the tag's location with zero rotation. */
  private Pose2d getTagWaypoint(int tagId) {
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

  /** Simple square pattern auto */
  private Command simpleSquareAuto() {
    // drive sequence
    SequentialCommandGroup driveSequence = new SequentialCommandGroup(
        m_swerve.pathfindToPose(k_squarePoint1, false),
        m_swerve.pathfindToPose(k_squarePoint2, false),
        m_swerve.pathfindToPose(k_squarePoint3, false),
        m_swerve.pathfindToPose(k_squarePoint4, false));

    // run turret tracking test
    Command turretTrackingCommand = run(() -> {
      m_vision.getTurretCamera().aimAtCenterOfTargets(m_swerve.getPose());
    });

    // Run turret tracking in parallel with driving, ending when drive completes
    return new ParallelDeadlineGroup(driveSequence, turretTrackingCommand);
  }

  /** The selected autonomous command. */
  public Command getAutonomousCommand() {
    return m_autochooser.getSelected();
  }

  /** Use for testing. Runs in robot periodic. */
  public void testRun() {
    
  }
}