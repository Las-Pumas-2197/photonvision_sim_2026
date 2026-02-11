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

public class Constants {
  public class SwerveDriveConstants {
    public static final double k_mass = Units.lbsToKilograms(115);
    public static final double k_wheelradius = Units.inchesToMeters(2);
    public static final double k_wheelcircumference = 2 * Math.PI * k_wheelradius;
    public static final double k_framelength = Units.inchesToMeters(33); // front to back (X)
    public static final double k_framewidth = Units.inchesToMeters(22); // side to side (Y)
    public static final double k_moduleradius = Math.hypot(k_framelength / 2, k_framewidth / 2);
    public static final double k_MOI = 0.5 * k_mass * Math.pow(k_moduleradius, 2);
    public static final double k_drivegearratio = 5.27;
    public static final double k_turngearratio = 26;
    public static final double k_drivemotormaxRPM = 6784;
    public static final double k_maxlinspeed = (k_drivemotormaxRPM / k_drivegearratio) * k_wheelcircumference / 60; // meters/sec
    public static final double k_maxrotspeed = (2 * k_maxlinspeed) / k_moduleradius;
    public static final Pose2d k_initpose = new Pose2d(2, 2, new Rotation2d());
  }

  public class OIConstants {
    public static final int k_joystickport = 0;
    public static final double k_maxlinspeedteleop = 3;
    public static final double k_maxrotspeedteleop = 2 * Math.PI;
  }

  public class VisionConstants {

    // list of camera instrinsics
    public static final List<Transform3d> k_cameraintrinsics = List.of(
        new Transform3d(
            Units.inchesToMeters(12),
            Units.inchesToMeters(12),
            Units.inchesToMeters(8),
            new Rotation3d(
                0,
                Units.degreesToRadians(-20),
                Units.degreesToRadians(45))),
        new Transform3d(
            Units.inchesToMeters(12),
            Units.inchesToMeters(-12),
            Units.inchesToMeters(8),
            new Rotation3d(
                0,
                Units.degreesToRadians(-20),
                Units.degreesToRadians(-45))),
        new Transform3d(
            Units.inchesToMeters(-12),
            Units.inchesToMeters(12),
            Units.inchesToMeters(8),
            new Rotation3d(
                0,
                Units.degreesToRadians(-20),
                Units.degreesToRadians(135))),
        new Transform3d(
            Units.inchesToMeters(-12),
            Units.inchesToMeters(-12),
            Units.inchesToMeters(8),
            new Rotation3d(
                0,
                Units.degreesToRadians(-20),
                Units.degreesToRadians(-135)))
    );

    // camera names
    public static final List<String> k_cameranames = List.of(
      "frontleft_camera",
      "frontright_camera",
      "rearleft_camera",
      "rearright_camera"
    );

    // apriltag layout
    public static final AprilTagFieldLayout k_fieldlayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);

    // standard deviations
    public static final Matrix<N3, N1> k_singletagstddevs = VecBuilder.fill(0.01, 0.01, 0.01);
    public static final Matrix<N3, N1> k_multitagstddevs = VecBuilder.fill(0.00, 0.00, 0.00);
    // public static final Matrix<N3, N1> k_multitagstddevs = VecBuilder.fill(0.005, 0.005, 0.005);
    public static final Matrix<N3, N1> k_ignorestddevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    // turret camera configuration
    public static final String k_turretcameraname = "turret_camera";
    public static final Transform3d k_turretbasetransform = new Transform3d(
        0, // centered on robot X
        0, // centered on robot Y
        Units.inchesToMeters(12), // elevated 12 inches
        new Rotation3d(0, Units.degreesToRadians(-15), 0) // tilted down 15 degrees
    );
    public static final List<Integer> k_turrettargetids = List.of(18, 27, 26, 25, 24, 21);
  }

  public static final class PathfindingConstants {
    public static final Pose2d k_bluereefA = new Pose2d(1, 4, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d k_redreefA = new Pose2d(16, 4, new Rotation2d(Units.degreesToRadians(0)));

    // Field center (field is 16.54m x 8.07m)
    public static final Pose2d k_fieldCenter = new Pose2d(8.27, 4.0, new Rotation2d(0));

    // Basin center for turret aiming
    public static final Pose2d k_basinCenter = new Pose2d(4.5, 4.0, new Rotation2d(0));

    // Simple square auto waypoints (starting from initial pose at 2,2)
    public static final Pose2d k_squarePoint1 = new Pose2d(1.5, 1, new Rotation2d(0));
    public static final Pose2d k_squarePoint2 = new Pose2d(1.5, 7, new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d k_squarePoint3 = new Pose2d(1, 8, new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d k_squarePoint4 = new Pose2d(2, 1, new Rotation2d(Units.degreesToRadians(270)));
  }
}
