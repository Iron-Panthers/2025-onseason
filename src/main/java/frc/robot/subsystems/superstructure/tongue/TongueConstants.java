package frc.robot.subsystems.superstructure.tongue;

import frc.robot.Constants;
import java.util.Optional;

public class TongueConstants {
  // FIXME
  public static final TongueConfig TONGUE_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new TongueConfig(0);
        case ALPHA -> new TongueConfig(15);
        case PROG -> new TongueConfig(0);
        case SIM -> new TongueConfig(0);
      };

  public record TongueConfig(int servoID) {}

  public static final boolean INVERT_MOTOR = true;

  public static final double POSITION_TARGET_EPSILON = 5;

  public static final double OFFSET = 0;

  // SOFT LIMITS
  public static final Optional<Double> UPPER_EXTENSION_LIMIT =
      Optional.empty(); // top limit is 121 rotations
  public static final Optional<Double> LOWER_EXTENSION_LIMIT =
      Optional.empty(); // top limit is 121 rotations

  // top limit is 121 rotations

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 3;
  public static final double LOWER_VOLT_LIMIT = -3;
  public static final double SUPPLY_CURRENT_LIMIT = 30;
}
