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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretCamera extends SubsystemBase {

  private final PhotonCamera m_camera;
  private PhotonCameraSim m_camerasim;
  private final Transform3d m_basetransform;
  private double m_turretyaw = 0.0; // current turret yaw in radians

  private Optional<PhotonPipelineResult> results = Optional.empty();

  public TurretCamera(String name, Transform3d baseTransform) {
    m_camera = new PhotonCamera(name);
    m_basetransform = baseTransform;

    if (RobotBase.isSimulation()) {
      SimCameraProperties camerasim_properties = new SimCameraProperties();
      camerasim_properties.setFPS(10);
      camerasim_properties.setCalibration(960, 720, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(70));
      m_camerasim = new PhotonCameraSim(m_camera, camerasim_properties);
      m_camerasim.enableRawStream(true);
      m_camerasim.enableDrawWireframe(true);
    }
  }

  /**
   * Gets the current turret transform including rotation.
   * @return The full Transform3d of the turret camera.
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
   * @param yawRadians The yaw angle in radians.
   */
  public void setTurretYaw(double yawRadians) {
    m_turretyaw = yawRadians;
  }

  /**
   * Gets the current turret yaw angle.
   * @return The yaw angle in radians.
   */
  public double getTurretYaw() {
    return m_turretyaw;
  }

  /**
   * Calculates the angle to aim at a specific fiducial from the robot pose.
   * @param robotPose The current robot pose.
   * @param fiducialId The ID of the fiducial to track.
   * @return The yaw angle in radians to aim at the target, or empty if tag not found.
   */
  public Optional<Double> calculateAngleToTarget(Pose2d robotPose, int fiducialId) {
    Optional<Pose3d> tagPose = k_fieldlayout.getTagPose(fiducialId);
    if (tagPose.isEmpty()) {
      return Optional.empty();
    }

    // Get robot position and tag position
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double tagX = tagPose.get().getX();
    double tagY = tagPose.get().getY();

    // Calculate angle from robot to tag in field coordinates
    double angleToTag = Math.atan2(tagY - robotY, tagX - robotX);

    // Convert to robot-relative angle (subtract robot heading)
    double robotHeading = robotPose.getRotation().getRadians();
    double turretAngle = angleToTag - robotHeading;

    // Normalize to -PI to PI
    while (turretAngle > Math.PI) turretAngle -= 2 * Math.PI;
    while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

    return Optional.of(turretAngle);
  }

  /**
   * Finds the closest visible target from the turret target list and calculates angle to it.
   * @param robotPose The current robot pose.
   * @return The yaw angle in radians to aim at the closest target, or empty if none visible.
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
   * Uses vector averaging to properly handle angle wraparound.
   * @param robotPose The current robot pose.
   * @return The center yaw angle in radians, or empty if no targets visible.
   */
  public Optional<Double> calculateAngleToCenterOfTargets(Pose2d robotPose) {
    List<Integer> visibleIds = getVisibleTargetIds();
    if (visibleIds.isEmpty()) {
      return Optional.empty();
    }

    // Use vector averaging to handle angle wraparound correctly
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

    // Average the vectors and convert back to angle
    double avgX = sumX / count;
    double avgY = sumY / count;
    return Optional.of(Math.atan2(avgY, avgX));
  }

  /**
   * Aims the turret at the interpolated center of all visible targets.
   * @param robotPose The current robot pose.
   */
  public void aimAtCenterOfTargets(Pose2d robotPose) {
    calculateAngleToCenterOfTargets(robotPose).ifPresent(this::setTurretYaw);
  }

  /**
   * Aims the turret at the closest target from the target list.
   * @param robotPose The current robot pose.
   */
  public void aimAtClosestTarget(Pose2d robotPose) {
    calculateAngleToClosestTarget(robotPose).ifPresent(this::setTurretYaw);
  }

  /**
   * Aims the turret at a specific fiducial.
   * @param robotPose The current robot pose.
   * @param fiducialId The ID of the fiducial to track.
   */
  public void aimAtTarget(Pose2d robotPose, int fiducialId) {
    calculateAngleToTarget(robotPose, fiducialId).ifPresent(this::setTurretYaw);
  }

  /**
   * Returns whether a target from the turret target list is currently visible.
   * @return True if any target is visible.
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
   * @return List of visible target IDs from the turret target list.
   */
  public List<Integer> getVisibleTargetIds() {
    if (results.isEmpty()) return List.of();
    return results.get().targets.stream()
        .map(t -> t.fiducialId)
        .filter(k_turrettargetids::contains)
        .toList();
  }

  /** Returns the most recent pipeline result. */
  public Optional<PhotonPipelineResult> getResults() {
    return results;
  }

  /** Returns the camera sim instance to interface with. */
  public PhotonCameraSim getSimInstance() {
    return m_camerasim;
  }

  /** Returns a boolean with the connection status of the camera. */
  public boolean isConnected() {
    return m_camera.isConnected();
  }

  @Override
  public void periodic() {
    for (var result : m_camera.getAllUnreadResults()) {
      results = Optional.of(result);
    }
  }
}
