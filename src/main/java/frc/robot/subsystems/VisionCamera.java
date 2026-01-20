// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.Constants.VisionConstants.k_fieldlayout;
import static frc.robot.utils.Constants.VisionConstants.k_ignorestddevs;
import static frc.robot.utils.Constants.VisionConstants.k_multitagstddevs;
import static frc.robot.utils.Constants.VisionConstants.k_singletagstddevs;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionCamera extends SubsystemBase {

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_estimator;
  private PhotonCameraSim m_camerasim;

  private Optional<PhotonPipelineResult> results = Optional.empty();
  private Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> estimate = Pair.of(Optional.empty(), k_ignorestddevs);
  private List<Integer> fiducials = new ArrayList<>();


  public VisionCamera(String name, Transform3d intrinsics) {

    m_camera = new PhotonCamera(name);
    m_estimator = new PhotonPoseEstimator(
        k_fieldlayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        intrinsics);
    m_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (RobotBase.isSimulation()) {
      SimCameraProperties camerasim_properties = new SimCameraProperties();
      camerasim_properties.setFPS(10);
      m_camerasim = new PhotonCameraSim(m_camera, camerasim_properties);
      m_camerasim.enableRawStream(true);
      m_camerasim.enableDrawWireframe(true);
    }
  }

  /**
   * Returns a list of the current visible fiducials.
   * 
   * @return List of integers containing the visible targets in the results.
   */
  public List<Integer> getFiducialIDs() {
    fiducials.clear();
    results.ifPresent(res -> {
      for (var target : res.targets) {
        fiducials.add(target.fiducialId);
      }
    });
    return fiducials;
  }

  /**
   * Use to calculate the std devs for the resultant estimate. Used internally only.
   * @param estimate The estimate.
   * @param result The results.
   * @return The calculated standard deviations.
   */
  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose estimate, PhotonPipelineResult result) {

    // zero out vars to recalculate avg distance and std devs
    Matrix<N3, N1> stddevs = k_ignorestddevs;
    int numTags = result.targets.size();
    double avgDist = 0.0;

    // calculate average target distance
    for (var target : result.targets) {
      Optional<Pose3d> target_pose = m_estimator.getFieldTags().getTagPose(target.fiducialId);
      if (target_pose.isEmpty()) continue;
      avgDist += target_pose.get().toPose2d().getTranslation().getDistance(estimate.estimatedPose.toPose2d().getTranslation());
    }
    avgDist /= numTags;

    // heuristic logic
    if (numTags > 1 && avgDist < 3) {
      stddevs = k_multitagstddevs;
    } else if (numTags > 1 && avgDist > 4) {
      stddevs = k_multitagstddevs.times(Math.pow(numTags, 2) * 0.1);
    } else {
      stddevs = k_singletagstddevs;
    }
    
    // return the composed std devs
    return stddevs;
  }

  /** Returns the most recent pipeline result. */
  public Optional<PhotonPipelineResult> getResults() {
    return results;
  }

  /** Returns the current estimate with standard deviations. */
  public Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> getEstimate() {
    return estimate;
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

    // clear results and update them
    for (var result : m_camera.getAllUnreadResults()) {
      results = Optional.of(result);
    }

    // check if results are present for each camera, if yes, perform heuristics and
    // compose estimate
    results.ifPresentOrElse(result -> {
      m_estimator.update(result).ifPresent(est -> {
        estimate = Pair.of(Optional.of(est), calculateStdDevs(est, result));
      });
    }, () -> estimate = Pair.of(Optional.empty(), k_ignorestddevs)); // empty estimate and set stddevs to
                                                                     // ignore
  }
}
