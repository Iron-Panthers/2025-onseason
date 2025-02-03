package frc.robot.subsystems.vision;

import static frc.robot.Constants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static final double AMBIGUITY_CUTOFF = 0.1;
  public static final double Z_ERROR_CUTOFF = 0.5;

  // index 0 -> arducam-1, etc
  public static final Transform3d[] CAMERA_TRANSFORM =
      switch (getRobotType()) {
        case COMP -> new Transform3d[] {};
        case ALPHA -> new Transform3d[] {
          new Transform3d(0.258, 0, 0.454, new Rotation3d(Math.PI, -0.6109, 0))
        };
        default -> new Transform3d[0];
      };

  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
}
