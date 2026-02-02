// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.Constants.VisionConstants.k_fieldlayout;
import static frc.robot.utils.Constants.VisionConstants.k_turrettargetids;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret camera that uses PhotonVision simulation backend but publishes
 * data to Limelight-style NetworkTables for compatibility.
 *
 * In simulation, view the camera stream at http://localhost:1182
 */
public class LimelightTurretCamera extends SubsystemBase {

  private final PhotonCamera m_camera;
  private PhotonCameraSim m_camerasim;
  private final Transform3d m_basetransform;
  private double m_turretyaw = 0.0;

  // Limelight NetworkTables
  private final NetworkTable m_limelightTable;
  private final NetworkTableEntry m_tv; // target valid
  private final NetworkTableEntry m_tx; // horizontal offset (degrees)
  private final NetworkTableEntry m_ty; // vertical offset (degrees)
  private final NetworkTableEntry m_ta; // target area (percentage)
  private final NetworkTableEntry m_tid; // fiducial ID

  private Optional<PhotonPipelineResult> results = Optional.empty();

  public LimelightTurretCamera(String name, Transform3d baseTransform) {
    m_camera = new PhotonCamera(name);
    m_basetransform = baseTransform;

    // Setup Limelight NetworkTables
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");
    m_tv = m_limelightTable.getEntry("tv");
    m_tx = m_limelightTable.getEntry("tx");
    m_ty = m_limelightTable.getEntry("ty");
    m_ta = m_limelightTable.getEntry("ta");
    m_tid = m_limelightTable.getEntry("tid");

    // Initialize NT values
    m_tv.setDouble(0.0);
    m_tx.setDouble(0.0);
    m_ty.setDouble(0.0);
    m_ta.setDouble(0.0);
    m_tid.setDouble(-1);

    if (RobotBase.isSimulation()) {
      SimCameraProperties camerasim_properties = new SimCameraProperties();
      camerasim_properties.setFPS(30);
      // 63° horizontal FOV, 43° vertical FOV → 72.14° diagonal FOV
      // Resolution aspect ratio (1120/720 = 1.556) matches FOV ratio (tan(31.5°)/tan(21.5°))
      camerasim_properties.setCalibration(1120, 720, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(72.14));
      m_camerasim = new PhotonCameraSim(m_camera, camerasim_properties);
      m_camerasim.enableRawStream(true);
      m_camerasim.enableDrawWireframe(true);
    }
  }

  /**
   * Gets the current turret transform including rotation.
   */
  public Transform3d getTurretTransform() {
    Rotation3d baseRotation = m_basetransform.getRotation();
    Rotation3d turretRotation = new Rotation3d(
        baseRotation.getX(),
        baseRotation.getY(),
        baseRotation.getZ() + m_turretyaw
    );
    return new Transform3d(m_basetransform.getTranslation(), turretRotation);
  }

  /**
   * Sets the turret yaw angle.
   */
  public void setTurretYaw(double yawRadians) {
    m_turretyaw = yawRadians;
  }

  /**
   * Gets the current turret yaw angle.
   */
  public double getTurretYaw() {
    return m_turretyaw;
  }

  // ===== Limelight-style getters (read from NetworkTables) =====

  /**
   * Returns whether a valid target is present (Limelight tv).
   */
  public boolean hasTarget() {
    return m_tv.getDouble(0.0) == 1.0;
  }

  /**
   * Gets horizontal offset to target in degrees (Limelight tx).
   */
  public double getTX() {
    return m_tx.getDouble(0.0);
  }

  /**
   * Gets vertical offset to target in degrees (Limelight ty).
   */
  public double getTY() {
    return m_ty.getDouble(0.0);
  }

  /**
   * Gets target area as percentage (Limelight ta).
   */
  public double getTA() {
    return m_ta.getDouble(0.0);
  }

  /**
   * Gets the fiducial ID of the primary target (Limelight tid).
   */
  public int getTID() {
    return (int) m_tid.getDouble(-1);
  }

  // ===== Turret aiming methods =====

  /**
   * Calculates the angle to aim at a specific fiducial from the robot pose.
   */
  public Optional<Double> calculateAngleToTarget(Pose2d robotPose, int fiducialId) {
    Optional<Pose3d> tagPose = k_fieldlayout.getTagPose(fiducialId);
    if (tagPose.isEmpty()) {
      return Optional.empty();
    }

    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double tagX = tagPose.get().getX();
    double tagY = tagPose.get().getY();

    double angleToTag = Math.atan2(tagY - robotY, tagX - robotX);
    double robotHeading = robotPose.getRotation().getRadians();
    double turretAngle = angleToTag - robotHeading;

    while (turretAngle > Math.PI) turretAngle -= 2 * Math.PI;
    while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

    return Optional.of(turretAngle);
  }

  /**
   * Finds the closest visible target and calculates angle to it.
   */
  public Optional<Double> calculateAngleToClosestTarget(Pose2d robotPose) {
    double closestDist = Double.MAX_VALUE;
    Optional<Double> closestAngle = Optional.empty();

    for (int tagId : k_turrettargetids) {
      Optional<Pose3d> tagPose = k_fieldlayout.getTagPose(tagId);
      if (tagPose.isEmpty()) continue;

      double dist = robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
      if (dist < closestDist) {
        closestDist = dist;
        closestAngle = calculateAngleToTarget(robotPose, tagId);
      }
    }

    return closestAngle;
  }

  /**
   * Calculates the interpolated center angle between all visible turret targets.
   */
  public Optional<Double> calculateAngleToCenterOfTargets(Pose2d robotPose) {
    List<Integer> visibleIds = getVisibleTargetIds();
    if (visibleIds.isEmpty()) {
      return Optional.empty();
    }

    double sumX = 0;
    double sumY = 0;
    int count = 0;

    for (int tagId : visibleIds) {
      Optional<Double> angle = calculateAngleToTarget(robotPose, tagId);
      if (angle.isPresent()) {
        sumX += Math.cos(angle.get());
        sumY += Math.sin(angle.get());
        count++;
      }
    }

    if (count == 0) {
      return Optional.empty();
    }

    double avgX = sumX / count;
    double avgY = sumY / count;
    return Optional.of(Math.atan2(avgY, avgX));
  }

  /**
   * Aims the turret at the interpolated center of all visible targets.
   */
  public void aimAtCenterOfTargets(Pose2d robotPose) {
    calculateAngleToCenterOfTargets(robotPose).ifPresent(this::setTurretYaw);
  }

  /**
   * Aims the turret at the closest target from the target list.
   */
  public void aimAtClosestTarget(Pose2d robotPose) {
    calculateAngleToClosestTarget(robotPose).ifPresent(this::setTurretYaw);
  }

  /**
   * Aims the turret at a specific fiducial.
   */
  public void aimAtTarget(Pose2d robotPose, int fiducialId) {
    calculateAngleToTarget(robotPose, fiducialId).ifPresent(this::setTurretYaw);
  }

  /**
   * Returns whether a target from the turret target list is currently visible.
   */
  public boolean hasVisibleTarget() {
    if (results.isEmpty()) return false;
    for (var target : results.get().targets) {
      if (k_turrettargetids.contains(target.fiducialId)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Returns the list of currently visible turret target IDs.
   */
  public List<Integer> getVisibleTargetIds() {
    if (results.isEmpty()) return List.of();
    return results.get().targets.stream()
        .map(t -> t.fiducialId)
        .filter(k_turrettargetids::contains)
        .toList();
  }

  /** Returns the camera sim instance. */
  public PhotonCameraSim getSimInstance() {
    return m_camerasim;
  }

  /** Returns connection status. */
  public boolean isConnected() {
    return m_camera.isConnected();
  }

  /** Updates Limelight NetworkTables from PhotonVision results. */
  private void updateLimelightNetworkTables() {
    if (results.isEmpty() || results.get().targets.isEmpty()) {
      m_tv.setDouble(0.0);
      m_tx.setDouble(0.0);
      m_ty.setDouble(0.0);
      m_ta.setDouble(0.0);
      m_tid.setDouble(-1);
      return;
    }

    // Find best target from turret target list
    PhotonTrackedTarget bestTarget = null;
    for (var target : results.get().targets) {
      if (k_turrettargetids.contains(target.fiducialId)) {
        if (bestTarget == null || target.getArea() > bestTarget.getArea()) {
          bestTarget = target;
        }
      }
    }

    if (bestTarget != null) {
      m_tv.setDouble(1.0);
      m_tx.setDouble(bestTarget.getYaw());
      m_ty.setDouble(bestTarget.getPitch());
      m_ta.setDouble(bestTarget.getArea());
      m_tid.setDouble(bestTarget.fiducialId);
    } else {
      m_tv.setDouble(0.0);
      m_tx.setDouble(0.0);
      m_ty.setDouble(0.0);
      m_ta.setDouble(0.0);
      m_tid.setDouble(-1);
    }
  }

  @Override
  public void periodic() {
    for (var result : m_camera.getAllUnreadResults()) {
      results = Optional.of(result);
    }
    updateLimelightNetworkTables();
  }
}
