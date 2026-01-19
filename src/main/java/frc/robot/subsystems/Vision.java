package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.robot.Constants.VisionConstants;

public class Vision {
  private final PoseEstimator3d<?> m_poseEstimator;
  private final PhotonCamera[] m_cameras = new PhotonCamera[VisionConstants.kCameraNames.length];

  private final PhotonPoseEstimator m_visionPoseEstimator = new PhotonPoseEstimator(
      VisionConstants.kAprilTagFieldLayout, Transform3d.kZero);

  public Vision(PoseEstimator3d<?> poseEstimator) {
    m_poseEstimator = poseEstimator;

    for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
      m_cameras[i] = new PhotonCamera(VisionConstants.kCameraNames[i]);
    }
  }

  private Optional<PhotonPipelineResult> getLatestResult(int cameraIndex) {
    List<PhotonPipelineResult> results = m_cameras[cameraIndex].getAllUnreadResults();

    if (results.isEmpty()) {
      return Optional.empty();
    } else {
      return Optional.of(results.get(results.size() - 1));
    }
  }

  private Optional<EstimatedRobotPose> getCameraPoseEstimate(int cameraIndex) {
    m_visionPoseEstimator
        .setRobotToCameraTransform(VisionConstants.kRobotToCameraTransforms[cameraIndex]);

    Optional<PhotonPipelineResult> latestResult = getLatestResult(cameraIndex);
    if (latestResult.isEmpty()) {
      return Optional.empty();
    }

    return m_visionPoseEstimator.estimateCoprocMultiTagPose(latestResult.get());
  }

  private double getAverageDistance(EstimatedRobotPose poseEstimate) {
    double totalDistance = 0.0;
    for (PhotonTrackedTarget target : poseEstimate.targetsUsed) {
      totalDistance += target.bestCameraToTarget.getTranslation().getNorm();
    }

    return totalDistance / poseEstimate.targetsUsed.size();
  }

  private Matrix<N4, N1> getCameraStdDevsForDistance(int cameraIndex, double distanceMeters) {
    return VisionConstants.kVisionMeasurementStdDevs.get(cameraIndex).times(distanceMeters);
  }

  public void update() {
    for (int i = 0; i < m_cameras.length; i++) {
      Optional<EstimatedRobotPose> poseEstimateOptional = getCameraPoseEstimate(i);
      if (poseEstimateOptional.isEmpty())
        continue;

      EstimatedRobotPose poseEstimate = poseEstimateOptional.get();

      double averageDistance = getAverageDistance(poseEstimate);

      m_poseEstimator.addVisionMeasurement(poseEstimate.estimatedPose,
          poseEstimate.timestampSeconds, getCameraStdDevsForDistance(i, averageDistance));
    }
  }
}
