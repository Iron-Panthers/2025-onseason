package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static final double AMBIGUITY_CUTOFF = 0.1; // FIXME
  public static final double Z_ERROR_CUTOFF = 0.5;

  // index 0 -> arducam-1, etc
  public static final Transform3d[] CAMERA_TRANSFORM = {
    // arducam-1 (front center)
    // new Transform3d(0.258, 0, 0.454, new Rotation3d(Math.PI, -Math.toRadians(35), 0)),
    new Transform3d(0.258, 0, 0.454, new Rotation3d(0, -Math.toRadians(35), 0)),

    // arducam-2 (front left)
    new Transform3d(
        0.258, 0.260, 0.454, new Rotation3d(0, -Math.toRadians(35), Math.toRadians(55))),

    // arducam-3 (back)
    new Transform3d(-0.1524, -0.0795, 0.2794, new Rotation3d(Math.PI, -Math.toRadians(160), 0))
  };

  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
}
