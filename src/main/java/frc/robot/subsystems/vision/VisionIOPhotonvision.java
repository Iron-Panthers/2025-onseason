package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonvision implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;

  public VisionIOPhotonvision(int index) {
    camera = new PhotonCamera("arducam-" + index);
    estimator =
        new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.CAMERA_TRANSFORM[index - 1]);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    List<PoseObservation> observations = new ArrayList<PoseObservation>();

    List<Integer> allTagIDs = new ArrayList<Integer>();

    for (int frameIndex = 0; frameIndex < results.size(); ++frameIndex) {
      PhotonPipelineResult frame = results.get(frameIndex);
      if (!frame.hasTargets()) continue;

      Optional<EstimatedRobotPose> optEstimation = estimator.update(frame);
      if (optEstimation.isEmpty()) continue;
      EstimatedRobotPose estimation = optEstimation.get();

      double totalDistance = 0;
      for (PhotonTrackedTarget target : frame.getTargets()) {
        totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
      }

      List<Integer> FIDs = new ArrayList<Integer>();
      for (PhotonTrackedTarget target : estimation.targetsUsed) {
        FIDs.add(target.getFiducialId());
      }
      allTagIDs.addAll(FIDs);

      var observation =
          new PoseObservation(
              frame.getTimestampSeconds(),
              estimation.estimatedPose,
              estimation.targetsUsed.get(0).poseAmbiguity,
              results.get(frameIndex).targets.size(),
              totalDistance / results.get(frameIndex).targets.size());
      observations.add(observation);
    }

    inputs.observations = observations.toArray(new PoseObservation[observations.size()]);

    inputs.tagIDs = allTagIDs.stream().mapToInt(i -> i).toArray();
  }
}
