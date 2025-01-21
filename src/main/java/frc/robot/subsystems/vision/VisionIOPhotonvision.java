package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
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
            new Transform3d()); // FIXME transform
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    PoseObservation[] observations = new PoseObservation[results.size()];

    List<Short> allTagIDs = new ArrayList<Short>();

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

      // FIXME
      List<Short> FIDs = frame.getMultiTagResult().get().fiducialIDsUsed;
      allTagIDs.addAll(FIDs);

      var observation =
          new PoseObservation(
              frame.getTimestampSeconds(),
              estimation.estimatedPose,
              frame.getMultiTagResult().get().estimatedPose.ambiguity,
              results.get(frameIndex).targets.size(),
              totalDistance / results.get(frameIndex).targets.size());
      observations[frameIndex] = observation;
    }

    inputs.observations = observations;

    inputs.tagIDs = new int[allTagIDs.size()];
    int i = 0;
    for (int id : allTagIDs) {
      inputs.tagIDs[i++] = id;
    }
  }
}
