package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Processes vision data from PhotonVision and updating the robot's pose estimate. */
public class Vision {
  private final PoseEstimator3d<?> m_poseEstimator;
  private final PhotonCamera[] m_cameras = new PhotonCamera[VisionConstants.kCameraNames.length];

  private final PhotonPoseEstimator m_visionPoseEstimator =
      new PhotonPoseEstimator(VisionConstants.kAprilTagFieldLayout, Transform3d.kZero);

  /** Create a new Vision. */
  public Vision(PoseEstimator3d<?> poseEstimator) {
    m_poseEstimator = poseEstimator;

    for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
      m_cameras[i] = new PhotonCamera(VisionConstants.kCameraNames[i]);
    }
  }

  /**
   * Gets the latest result from the specified camera.
   *
   * @param cameraIndex The index of the camera.
   * @return The latest result from the specified camera.
   */
  private Optional<PhotonPipelineResult> getLatestResult(int cameraIndex) {
    List<PhotonPipelineResult> results = m_cameras[cameraIndex].getAllUnreadResults();

    if (results.isEmpty()) {
      return Optional.empty();
    } else {
      return Optional.of(results.get(results.size() - 1));
    }
  }

  /**
   * Gets the pose estimate from the specified camera.
   *
   * @param cameraIndex The index of the camera.
   * @return The pose estimate from the specified camera.
   */
  private Optional<EstimatedRobotPose> getCameraPoseEstimate(int cameraIndex) {
    m_visionPoseEstimator.setRobotToCameraTransform(
        VisionConstants.kRobotToCameraTransforms[cameraIndex]);

    Optional<PhotonPipelineResult> latestResult = getLatestResult(cameraIndex);
    if (latestResult.isEmpty()) {
      return Optional.empty();
    }

    return m_visionPoseEstimator.estimateCoprocMultiTagPose(latestResult.get());
  }

  /**
   * Gets the distance to the best target from the pose estimate.
   *
   * @param poseEstimate The pose estimate.
   * @return The distance to the best target.
   */
  private double getBestTargetDistance(EstimatedRobotPose poseEstimate) {
    // Best target is the first target in the list
    PhotonTrackedTarget target = poseEstimate.targetsUsed.get(0);
    return target.bestCameraToTarget.getTranslation().getNorm();
  }

  /**
   * Gets the standard deviations for the camera given a pose estimate.
   *
   * @param cameraIndex The index of the camera.
   * @param poseEstimate The pose estimate.
   * @return The scaled standard deviations.
   */
  private Matrix<N4, N1> getCameraStdDevs(int cameraIndex, EstimatedRobotPose poseEstimate) {
    Matrix<N4, N1> stdDevs = VisionConstants.kVisionMeasurementStdDevs.get(cameraIndex);

    double angleStdDev = stdDevs.get(3, 0);

    // Scale position std devs based on distance to target
    stdDevs = stdDevs.times(getBestTargetDistance(poseEstimate));
    stdDevs.set(3, 0, angleStdDev);

    return stdDevs;
  }

  /** Updates the pose estimator with vision measurements. */
  public void update() {
    for (int i = 0; i < m_cameras.length; i++) {
      Optional<EstimatedRobotPose> poseEstimateOptional = getCameraPoseEstimate(i);
      if (poseEstimateOptional.isEmpty()) continue;

      EstimatedRobotPose poseEstimate = poseEstimateOptional.get();

      m_poseEstimator.addVisionMeasurement(
          poseEstimate.estimatedPose,
          poseEstimate.timestampSeconds,
          getCameraStdDevs(i, poseEstimate));
    }
  }
}
