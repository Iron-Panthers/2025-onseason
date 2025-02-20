// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swerve.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

/* based on wpimath/../PoseEstimator.java */
public class RobotState {
  public static final double fieldSizeX = Units.feetToMeters(57.573);
  public static final double fieldSizeY = Units.feetToMeters(26.417);

  public record OdometryMeasurement(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record VisionMeasurement(Pose2d visionPose, double timestamp) {}

  private static final double poseBufferSizeSeconds = 2; // shorter?
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Pose2d initialPose =
      DriverStation.getAlliance().get() == Alliance.Red
          ? FlippingUtil.flipFieldPose(DriveConstants.INITAL_POSE)
          : DriveConstants.INITAL_POSE;

  private final Matrix<N3, N1> matrixQ = new Matrix<>(Nat.N3(), Nat.N1());
  private final Matrix<N3, N3> kalmanGain = new Matrix<>(Nat.N3(), Nat.N3());

  private TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  private Pose2d odometryPose = initialPose;
  private Pose2d estimatedPose = initialPose; // vision adjusted

  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = new Rotation2d();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      matrixQ.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }
  }

  /* standard deviations in [x, y, theta], SI units */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> stdDevs) {
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = stdDevs.get(i, 0) * stdDevs.get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    for (int row = 0; row < 3; ++row) {
      if (matrixQ.get(row, 0) == 0.0) {
        kalmanGain.set(row, row, 0.0);
      } else {
        kalmanGain.set(
            row,
            row,
            matrixQ.get(row, 0) / (matrixQ.get(row, 0) + Math.sqrt(matrixQ.get(row, 0) * r[row])));
      }
    }
  }

  /* update pose estimation based on odometry measurements, based on wpimath */
  public void addOdometryMeasurement(OdometryMeasurement measurement) {
    Twist2d twist =
        DriveConstants.KINEMATICS.toTwist2d(lastWheelPositions, measurement.wheelPositions());
    twist.dtheta = measurement.gyroAngle().minus(lastGyroAngle).getRadians();

    lastWheelPositions = measurement.wheelPositions();
    lastGyroAngle = measurement.gyroAngle();

    // integrate to find difference in pose over time, add to pose estimate
    odometryPose = odometryPose.exp(twist);
    estimatedPose = estimatedPose.exp(twist);

    // add post estimate to buffer at timestamp; for vision
    poseBuffer.addSample(measurement.timestamp(), odometryPose);
  }

  /* from wpimath PoseEstimator.java */
  public void addVisionMeasurement(VisionMeasurement measurement) {
    // if measurement is old enough to be outside buffer timespan, skip
    if (poseBuffer.getInternalBuffer().isEmpty()
        || poseBuffer.getInternalBuffer().lastKey() < poseBufferSizeSeconds) {
      return;
    }

    // get odometry pose from moment of vision measurement
    var sample = poseBuffer.getSample(measurement.timestamp());
    if (sample.isEmpty()) return;

    // twists to get from sampled <--> current odometry pose
    var sampleToOdometry = sample.get().log(odometryPose);
    var odometryToSample = odometryPose.log(sample.get());
    // calculate old estimate
    Pose2d oldEstimate = estimatedPose.exp(odometryToSample);

    // measure twist between estimate and vision pose
    var twist = oldEstimate.log(measurement.visionPose());

    // scale twist by Kalman gain matrix; represents how much to trust vision vs. current pose
    var timesTwist = kalmanGain.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

    // convert back to Twist2d
    var scaledTwist = new Twist2d(timesTwist.get(0, 0), timesTwist.get(1, 0), timesTwist.get(2, 0));

    // apply Kalman-scaled vision adjustment, replay odometry data to get current estimate
    estimatedPose = sample.get().exp(scaledTwist).exp(sampleToOdometry);
  }

  public void addVisionMeasurement(VisionMeasurement measurement, Matrix<N3, N1> visionStdDevs) {
    setVisionMeasurementStdDevs(visionStdDevs);
    addVisionMeasurement(measurement);
  }

  public void resetPose(Pose2d pose) {
    odometryPose = pose;
    estimatedPose = pose;
    poseBuffer.clear();
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }
}
