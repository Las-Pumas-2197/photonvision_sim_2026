// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.Constants.SwerveDriveConstants.*;

import java.io.File;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {

  private final SwerveDrive m_swervedrive;
  private final RobotConfig m_PPconfig;

  public Swerve() {

    // attempt construction of YAGSL swerve object
    try {
      m_swervedrive = new SwerveParser(
          new File(Filesystem.getDeployDirectory(), "swerve"))
          .createSwerveDrive(k_maxlinspeed, k_initpose);
    } catch (Exception e) {
      // throw exception if directory is invalid
      throw new RuntimeException(e);
    }

    // swerve system options
    m_swervedrive.setMotorIdleMode(true);

    // attempt construction of PP config
    try {
      m_PPconfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // throw exception if directory is invalid
      throw new RuntimeException(e);
    }
  }

  /**
   * Configures the PP AutoBuilder with the robot intrinsics. Call before
   * constructing any autos.
   */
  public void runAutoBuilder() {
    AutoBuilder.configure(
        m_swervedrive::getPose, // Robot pose supplier
        m_swervedrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        m_swervedrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> m_swervedrive.drive(speeds, false, new Translation2d()), // Method that will drive the
                                                                                           // robot given ROBOT RELATIVE
        // ChassisSpeeds. Also optionally outputs individual module
        // feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(8, 0.0, 0.0), // Translation PID constants
            new PIDConstants(8, 0.0, 0.0) // Rotation PID constants
        ),
        m_PPconfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  /** Returns the current swerve drive instance. */
  public Field2d getField2d() {
    return m_swervedrive.field;
  }

  /** Returns the current field-relative chassis speeds. */
  public ChassisSpeeds getFieldSpeeds() {
    return m_swervedrive.getFieldVelocity();
  }

  /**
   * Basic field oriented driving method.
   * 
   * @param speeds Composed field-relative speeds.
   */
  public void drive(ChassisSpeeds speeds) {
    m_swervedrive.driveFieldOriented(speeds);
  }

  /** Returns the current pose of the robot. */
  public Pose2d getPose() {
    return m_swervedrive.getPose();
  }

  public Rotation2d getGyroHeading() {
    return m_swervedrive.getYaw();
  }

  /**
   * Returns the sim factory pose. Use only in simulation to update the vision sim
   * factory.
   */
  public Pose2d getSimPose() {
    if (m_swervedrive.getSimulationDriveTrainPose().isPresent()) {
      return m_swervedrive.getSimulationDriveTrainPose().get();
    } else {
      return new Pose2d();
    }
  }

  /**
   * Returns the maple-sim SwerveDriveSimulation for physics interaction.
   * Use this to access the robot's physics body in the SimulatedArena.
   */
  public Optional<swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation> getMapleSimDrive() {
    return m_swervedrive.getMapleSimDrive();
  }

  /** Adds the current queue of vision measurements to the pose estimator. */
  public void addVisionMeasurements(List<Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>>> estimates) {
    for (var estimate : estimates) {
      estimate.getFirst().ifPresent(est -> {
        m_swervedrive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estimate.getSecond());
      });
    }
  }

  /** Pathfinds to a specifed pose.
   *
   * @param targetpose The pose to pathfind to.
   * @param unlimited Pass true if unlimited constraints are requested.
   * @return The composed pathfinding command.
   */
  public Command pathfindToPose(Pose2d targetpose, boolean unlimited) {
    return pathfindToPose(targetpose, unlimited, 0.0);
  }

  /** Pathfinds to a specifed pose with goal end velocity.
   *
   * @param targetpose The pose to pathfind to.
   * @param unlimited Pass true if unlimited constraints are requested.
   * @param goalEndVelocity The velocity the robot should have when reaching the goal (m/s).
   *                        Use non-zero to pass through waypoints without stopping.
   * @return The composed pathfinding command.
   */
  public Command pathfindToPose(Pose2d targetpose, boolean unlimited, double goalEndVelocity) {
    if (unlimited) {
      return AutoBuilder.pathfindToPose(
          targetpose,
          PathConstraints.unlimitedConstraints(12),
          goalEndVelocity);
    } else {
      return AutoBuilder.pathfindToPose(
          targetpose,
          new PathConstraints(
              3,
              6,
              Math.PI * 4,
              Math.PI * 8),
          goalEndVelocity);
    }
  }

  @Override
  public void periodic() {
    m_swervedrive.updateOdometry();
  }
}
