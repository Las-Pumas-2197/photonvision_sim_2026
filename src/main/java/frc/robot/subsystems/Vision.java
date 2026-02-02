// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.utils.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

public class Vision extends SubsystemBase {

  /** Cameraaaaaaas. */
  private final List<VisionCamera> m_cameras = new ArrayList<>();

  /** Limelight-style turret camera for tracking base tags. */
  private final LimelightTurretCamera m_turretcamera;

  private VisionSystemSim m_visionsim;

  public Vision() {

    // iterate to instantiate cameras
    for (int num = 0; num < k_cameranames.size(); num++) {
      m_cameras.add(new VisionCamera(k_cameranames.get(num), k_cameraintrinsics.get(num)));
    }

    // instantiate limelight-style turret camera
    m_turretcamera = new LimelightTurretCamera(k_turretcameraname, k_turretbasetransform);

    // IT'S SIMULATIN TIME
    // declare vision sim, iterate and add cameras, add fiducials to sim field
    if (RobotBase.isSimulation()) {
      m_visionsim = new VisionSystemSim("main");
      for (int num = 0; num < m_cameras.size(); num++) {
        m_visionsim.addCamera(m_cameras.get(num).getSimInstance(), k_cameraintrinsics.get(num));
      }
      // add turret camera to sim
      m_visionsim.addCamera(m_turretcamera.getSimInstance(), m_turretcamera.getTurretTransform());
      m_visionsim.addAprilTags(k_fieldlayout);
    }
  }

  /** Get a list of all of the estimates from the camera pipelines. */
  public List<Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>>> getEstimates() {
    List<Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>>> estimates = new ArrayList<>();
    for (var camera : m_cameras) {
      if (camera.getEstimate().getFirst().isPresent()) {
        estimates.add(camera.getEstimate());
      }
    }
    return estimates;
  }

  /** Get a list of all the results from the camera pipelines. */
  public List<Optional<PhotonPipelineResult>> getResults() {
    List<Optional<PhotonPipelineResult>> results = new ArrayList<>();
    for (var camera : m_cameras) {
      if (camera.getResults().isPresent()) {
        results.add(camera.getResults());
      }
    }
    return results;
  }

  /** Returns a list containing all the visible fiducial IDs. */
  public List<Integer> getAllFiducialIDs() {
    HashSet<Integer> fiducials = new HashSet<>();
    for (var camera : m_cameras) {
      fiducials.addAll(camera.getFiducialIDs());
    }
    return fiducials.stream().toList();
  }

  /** Gets the yaw of a specific target seen by a specific camera index.
   * 
   * @param fiducialID The ID of the fidicuial whose yaw is requested.
   * @param cameraID The index of the camera to check for the fiducial.
   * @return The yaw as a Rotation2d. Returns an empty Rotation2d if fiducial is not visible.
  */
  public Rotation2d getCameraTargetYaw(int fiducialID, int cameraID) {
    Optional<PhotonPipelineResult> result = m_cameras.get(cameraID).getResults();
    Rotation2d target_yaw = new Rotation2d();
    if (result.isPresent()) {
      for (var target : result.get().targets) {
        if (target.fiducialId == fiducialID) {
          target_yaw = Rotation2d.fromDegrees(target.getYaw());
        }
      }
    }
    return target_yaw;
  }

  /** Returns the linear distance to the target. Target must be visible or it will return a distance of zero.
   * 
   * @param fiducialID The ID of the fiducial.
   * @param cameraID The ID of the camera where the fiducial is visible.
   * @return
   */
  public double getCameraTargetDistance(int fiducialID, int cameraID) {
    Optional<PhotonPipelineResult> result = m_cameras.get(cameraID).getResults();
    double target_distance = 0;
    if (result.isPresent()) {
      for (var target : result.get().targets) {
        if (target.fiducialId == fiducialID) {
          target_distance = target.getBestCameraToTarget().getTranslation().plus(k_cameraintrinsics.get(cameraID).getTranslation()).getDistance(new Translation3d());
        }
      }
    }
    return target_distance;
  }

  /** Returns a list of the vision system camera instances. */
  public List<VisionCamera> getCameras() {
    return m_cameras;
  }

  /** Returns the turret camera instance. */
  public LimelightTurretCamera getTurretCamera() {
    return m_turretcamera;
  }

  /** Used to update the pose of the vision sim periodically.
   *
   * @param pose The simulation's physical pose of the robot.
   */
  public void updatePose(Pose2d pose) {
    // update turret camera transform based on current turret angle
    if (RobotBase.isSimulation()) {
      m_visionsim.adjustCamera(m_turretcamera.getSimInstance(), m_turretcamera.getTurretTransform());
    }
    m_visionsim.update(pose);
  }

  @Override
  public void periodic() {

  }
}
