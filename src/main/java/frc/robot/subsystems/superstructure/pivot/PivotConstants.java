package frc.robot.subsystems.superstructure.pivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants;
import java.util.Optional;

public class PivotConstants {
  public static final PivotConfig PIVOT_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new PivotConfig(
            0,
            Optional.empty(),
            24.0 / 64.0); // TODO: add CANCoder id for comp bot pivot motor and encoder id for
          // CANCoder on pivot motor for comp bot and reduction for comp bot pivot
          // motor and encoder for comp bot pivot motor and reduction for comp bot
          // pivot motor based on comp bot pivot motor and encoder for CANCoder on
          // pivot motor for comp bot and reduction for CANCoder on pivot motor for
          // comp bot constants in Constants.java and Constants.java for comp bot and
          // comp bot pivot motor and encoder for CANCoder on pivot motor for comp
          // bot and reduction for CANCoder on pivot motor for comp bot and comp bot
          // pivot motor and encoder for CANCoder on pivot motor for comp bot and
          // reduction for CANCoder on pivot motor for comp bot data in
          // Constants.java for comp bot and comp bot pivot motor and encoder for
          // CANCoder on pivot motor for comp bot and reduction for CANCoder on pivot
          // motor for comp bot and comp bot pivot motor and encoder for CANCoder on
          // pivot motor for comp bot and reduction for CANCoder on pivot motor for
          // comp bot data in Constants.java for comp bot and comp bot pivot motor
          // and encoder for CANCoder on pivot motor for comp bot and reduction for
          // CANCoder on pivot motor for comp bot etc. for comp bot pivot motor and
          // encoder for CANCoder on pivot motor for comp bot and reduction for
          // CANCoder on pivot motor for comp bot and comp bot pivot motor and
          // encoder for CANCoder on pivot motor for comp bot and reduction for
          // CANCoder on pivot motor for comp bot for comp bot pivot motor and
          // encoder for CANCoder on pivot motor for comp bot and reduction for
          // CANCoder on pivot motor for comp bot and comp bot pivot motor and
          // encoder for CANCoder on pivot motor for comp bot and reduction for
          // CANCoder on pivot motor for comp bot get methods in Constants.java for
          // comp bot and comp bot pivot motor and encoder for CANCoder on pivot
          // motor for comp bot and reduction for CANCoder on pivot motor for comp
          // bot and comp bot pivot motor and encoder for CANCoder on pivot motor for
          // comp bot and reduction for CANCoder on pivot motor for comp bot here and
          // in Constants.java for comp bot and comp bot pivot motor and encoder for
          // CANCoder on pivot motor for comp bot and reduction for CANCoder on pivot
          // motor for comp bot and comp bot pivot motor and encoder for CANCoder on
          // pivot motor for comp bot and reduction for CANCoder on pivot motor for
          // comp bot i guess for comp bot pivot motor and encoder for CANCoder on
          // pivot motor for comp bot and reduction for CANCoder on pivot motor for
          // comp bot and comp bot pivot motor and encoder for CANCoder on pivot
          // motor for comp bot and reduction for CANCoder on pivot motor for comp
          // bot and comp bot pivot motor and encoder for CANCoder on pivot motor for
          // comp bot and reduction for CANCoder on pivot motor for comp bot just for
          // comp bot pivot motor and encoder for CANCoder on pivot motor for comp
          // bot and reduction for CANCoder on pivot motor for comp bot and comp bot
          // pivot motor and encoder for CANCoder on pivot motor for comp bot and
          // reduction for CANCoder on pivot motor for comp bot kthxbye
        case ALPHA -> new PivotConfig(15, Optional.empty(), 21.6 / 360);
        case PROG -> new PivotConfig(0, Optional.empty(), 1);
        case SIM -> new PivotConfig(0, Optional.empty(), 1);
      };
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case ALPHA -> new PIDGains(1.5, 0, 0.01, 0.03, 0.09, 0, 0.51);
        case PROG -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case SIM -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public record PivotConfig(int motorID, Optional<Integer> canCoderID, double reduction) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final boolean INVERT_MOTOR = true;

  public static final double POSITION_TARGET_EPSILON = 5;

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

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = 1;
  public static final double ZEROING_OFFSET = 110; // offset in degrees
  public static final double ZEROING_VOLTAGE_THRESHOLD = 5;
}
