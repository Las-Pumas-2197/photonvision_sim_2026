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
    // m_autochooser.setDefaultOption("no auto", print("WARNING: no auto scheduled"));
    m_autochooser.setDefaultOption("simple square", simpleSquareAuto());

    m_autochooser.addOption("autoalign reef A", autoAlignReef(18));
    m_autochooser.addOption("simple square", simpleSquareAuto());
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

  /** Composed default command for swerve subsystem. */
  private Command swerveDefaultCommand() {
    return new RunCommand(
        () -> m_swerve.drive(new ChassisSpeeds(
            MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * k_maxlinspeedteleop,
            -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * k_maxlinspeedteleop,
            -MathUtil.applyDeadband(m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop)),
        m_swerve);
  }

  /** Autoalign sequence for the reef. */
  private SequentialCommandGroup autoAlignReef(int fiducialID) {
    return new SequentialCommandGroup(
        m_swerve.pathfindToPose(k_bluereefA, true),
        m_swerve.pathfindToPose(k_redreefA, true));
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