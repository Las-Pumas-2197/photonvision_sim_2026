// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

  private Constants() {}

  /**
   * CAN IDs for all swerve module motors and encoders.
   * Ensure these match your actual hardware configuration.
   */
  public static final class CANIDs {
    // Front Left Module
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_LEFT_TURN = 2;
    public static final int FRONT_LEFT_CANCODER = 9;

    // Front Right Module
    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final int FRONT_RIGHT_TURN = 4;
    public static final int FRONT_RIGHT_CANCODER = 10;

    // Back Left Module
    public static final int BACK_LEFT_DRIVE = 5;
    public static final int BACK_LEFT_TURN = 6;
    public static final int BACK_LEFT_CANCODER = 11;

    // Back Right Module
    public static final int BACK_RIGHT_DRIVE = 7;
    public static final int BACK_RIGHT_TURN = 8;
    public static final int BACK_RIGHT_CANCODER = 12;
  }

  /**
   * SDS MK5i L2 swerve module constants.
   */
  public static final class ModuleConstants {
    // Gear ratios (MK5i L2)
    public static final double kDrivingMotorReduction = 6.75;
    public static final double kTurningMotorReduction = 150.0 / 7.0; // ~21.43:1

    // Wheel specifications
    public static final double kWheelDiameterInches = 4.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
    public static final double kWheelCircumferenceMeters = Math.PI * kWheelDiameterMeters;

    // NEO motor specifications
    public static final double kNeoFreeSpeedRpm = 5676.0;
    public static final double kDriveWheelFreeSpeedRps =
        (kNeoFreeSpeedRpm / 60.0) / kDrivingMotorReduction;

    // Theoretical max speed (m/s)
    public static final double kMaxDriveSpeed = kDriveWheelFreeSpeedRps * kWheelCircumferenceMeters;

    // Current limits
    public static final int kDriveCurrentLimit = 40;
    public static final int kTurnCurrentLimit = 20;

    // Encoder offsets (degrees) - CALIBRATE THESE ON YOUR ROBOT!
    public static final double kFrontLeftEncoderOffset = 0.0;
    public static final double kFrontRightEncoderOffset = 0.0;
    public static final double kBackLeftEncoderOffset = 0.0;
    public static final double kBackRightEncoderOffset = 0.0;

    // Turning encoder inverted (typically true for MK5i)
    public static final boolean kTurningEncoderInverted = true;
  }

  /**
   * Robot chassis and drivetrain constants.
   * Note: Uses k_ prefix to maintain compatibility with existing code.
   */
  public static final class SwerveDriveConstants {
    // Robot physical dimensions (wheel center to wheel center)
    public static final double k_trackwidth = Units.inchesToMeters(22.0);
    public static final double k_wheelbase = Units.inchesToMeters(22.0);
    public static final double k_framelength = k_wheelbase; // front to back (X)
    public static final double k_framewidth = k_trackwidth; // side to side (Y)

    // Robot mass
    public static final double k_mass = Units.lbsToKilograms(115.0);

    // Wheel specifications (from ModuleConstants)
    public static final double k_wheelradius = ModuleConstants.kWheelDiameterMeters / 2.0;
    public static final double k_wheelcircumference = ModuleConstants.kWheelCircumferenceMeters;

    // Gear ratios
    public static final double k_drivegearratio = ModuleConstants.kDrivingMotorReduction;
    public static final double k_turngearratio = ModuleConstants.kTurningMotorReduction;

    // Motor specs
    public static final double k_drivemotormaxRPM = ModuleConstants.kNeoFreeSpeedRpm;

    // Derived constants
    public static final double k_moduleradius = Math.hypot(k_framelength / 2, k_framewidth / 2);
    public static final double k_MOI = 0.5 * k_mass * Math.pow(k_moduleradius, 2);
    public static final double k_maxlinspeed = ModuleConstants.kMaxDriveSpeed;
    public static final double k_maxrotspeed = (2 * k_maxlinspeed) / k_moduleradius;

    // Initial pose
    public static final Pose2d k_initpose = new Pose2d(2, 2, new Rotation2d());
  }

  /**
   * Operator interface constants.
   */
  public static final class OIConstants {
    public static final int k_joystickport = 0;
    public static final double k_maxlinspeedteleop = 3.0; // m/s
    public static final double k_maxrotspeedteleop = 2 * Math.PI; // rad/s
  }

  /**
   * Vision system constants.
   */
  public static final class VisionConstants {
    // Camera transforms (position relative to robot center)
    public static final List<Transform3d> k_cameraintrinsics = List.of(
        new Transform3d(
            Units.inchesToMeters(12),
            Units.inchesToMeters(12),
            Units.inchesToMeters(8),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(45))),
        new Transform3d(
            Units.inchesToMeters(12),
            Units.inchesToMeters(-12),
            Units.inchesToMeters(8),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-45))),
        new Transform3d(
            Units.inchesToMeters(-12),
            Units.inchesToMeters(12),
            Units.inchesToMeters(8),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(135))),
        new Transform3d(
            Units.inchesToMeters(-12),
            Units.inchesToMeters(-12),
            Units.inchesToMeters(8),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-135)))
    );

    // Camera names
    public static final List<String> k_cameranames = List.of(
        "frontleft_camera",
        "frontright_camera",
        "rearleft_camera",
        "rearright_camera"
    );

    // AprilTag layout
    public static final AprilTagFieldLayout k_fieldlayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);

    // Standard deviations for pose estimation
    public static final Matrix<N3, N1> k_singletagstddevs = VecBuilder.fill(0.01, 0.01, 0.01);
    public static final Matrix<N3, N1> k_multitagstddevs = VecBuilder.fill(0.00, 0.00, 0.00);
    public static final Matrix<N3, N1> k_ignorestddevs = VecBuilder.fill(
        Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    // Turret camera configuration
    public static final String k_turretcameraname = "turret_camera";
    public static final Transform3d k_turretbasetransform = new Transform3d(
        0, 0, Units.inchesToMeters(12),
        new Rotation3d(0, Units.degreesToRadians(-15), 0)
    );
    public static final List<Integer> k_turrettargetids = List.of(18, 27, 26, 25, 24, 21);
  }

  /**
   * Pathfinding and autonomous constants.
   */
  public static final class PathfindingConstants {
    public static final Pose2d k_bluereefA = new Pose2d(1, 4, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d k_redreefA = new Pose2d(16, 4, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d k_fieldCenter = new Pose2d(8.27, 4.0, new Rotation2d(0));
    public static final Pose2d k_basinCenter = new Pose2d(4.5, 4.0, new Rotation2d(0));

    // Simple square auto waypoints
    public static final Pose2d k_squarePoint1 = new Pose2d(1.5, 1, new Rotation2d(0));
    public static final Pose2d k_squarePoint2 = new Pose2d(1.5, 7, new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d k_squarePoint3 = new Pose2d(1, 8, new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d k_squarePoint4 = new Pose2d(2, 1, new Rotation2d(Units.degreesToRadians(270)));
  }

  /**
   * Obstacle detection constants.
   */
  public static final class ObstacleDetectionConstants {
    public static final String k_frontCameraName = "limelight-front";
    public static final Transform3d k_frontCameraTransform = new Transform3d(
        Units.inchesToMeters(14), 0, Units.inchesToMeters(10),
        new Rotation3d(0, Math.toRadians(-5), 0)
    );

    public static final double k_detectionRange = 5.0;       // meters
    public static final double k_minConfidence = 0.5;
    public static final double k_obstacleStaleTime = 0.5;    // seconds
    public static final double k_robotClearance = 0.6;       // meters
    public static final double k_fuelClearance = 0.2;        // meters

    // Tunnel corridors
    public static final double k_northCorridorMinY = 6.5;
    public static final double k_northCorridorMaxY = 7.5;
    public static final double k_southCorridorMinY = 0.5;
    public static final double k_southCorridorMaxY = 1.5;
    public static final double k_tunnelMinX = 1.5;
    public static final double k_tunnelMaxX = 8.0;
  }
}
