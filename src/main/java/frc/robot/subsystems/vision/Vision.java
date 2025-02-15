package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  public Vision(VisionIO... io) {
    this.io = io;
    inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; ++i) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; ++i) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera " + i, inputs[i]);
    }

    // values for logging
    List<Pose3d> allTagPoses = new ArrayList<Pose3d>();
    List<Pose3d> allEstimatedPoses = new ArrayList<Pose3d>();
    List<Pose3d> allAcceptedPoses = new ArrayList<Pose3d>();
    List<Pose3d> allRejectedPoses = new ArrayList<Pose3d>();

    for (int cameraIndex = 0; cameraIndex < io.length; ++cameraIndex) {

      // logging
      List<Pose3d> tagPoses = new ArrayList<Pose3d>();
      List<Pose3d> estimatedPoses = new ArrayList<Pose3d>();
      List<Pose3d> acceptedPoses = new ArrayList<Pose3d>();
      List<Pose3d> rejectedPoses = new ArrayList<Pose3d>();

      for (int id : inputs[cameraIndex].tagIDs) {
        Optional<Pose3d> tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(id);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      PoseObservation[] observations = inputs[cameraIndex].observations;
      for (var observation : observations) {
        boolean rejectPose =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > AMBIGUITY_CUTOFF)
                || (observation.ambiguity() == -1 && observation.tagCount() == 1)
                || Math.abs(observation.estimatedPose().getZ())
                    > Z_ERROR_CUTOFF // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.estimatedPose().getX() < 0.0
                || observation.estimatedPose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength()
                || observation.estimatedPose().getY() < 0.0
                || observation.estimatedPose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

        // log poses
        estimatedPoses.add(observation.estimatedPose());

        if (rejectPose) rejectedPoses.add(observation.estimatedPose());
        else acceptedPoses.add(observation.estimatedPose());

        if (rejectPose) continue;

        var measurement =
            new VisionMeasurement(observation.estimatedPose().toPose2d(), observation.timestamp());

        Matrix<N3, N1> visionStdDevs =
            TAG_COUNT_DEVIATIONS
                .get(MathUtil.clamp(observation.tagCount() - 1, 0, TAG_COUNT_DEVIATIONS.size() - 1))
                .computeDeviation(observation.averageDistance());

        RobotState.getInstance().addVisionMeasurement(measurement, visionStdDevs);
      }

      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/estimatedPoses",
          estimatedPoses.toArray(new Pose3d[estimatedPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/AcceptedPoses",
          acceptedPoses.toArray(new Pose3d[acceptedPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RejectedPoses",
          rejectedPoses.toArray(new Pose3d[rejectedPoses.size()]));

      allTagPoses.addAll(tagPoses);
      allEstimatedPoses.addAll(estimatedPoses);
      allAcceptedPoses.addAll(acceptedPoses);
      allRejectedPoses.addAll(rejectedPoses);
    }

    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/estimatedPoses",
        allEstimatedPoses.toArray(new Pose3d[allEstimatedPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/AcceptedPoses",
        allAcceptedPoses.toArray(new Pose3d[allAcceptedPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RejectedPoses",
        allRejectedPoses.toArray(new Pose3d[allRejectedPoses.size()]));
  }
}
