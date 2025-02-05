package frc.robot.subsystems.superstructure.tongue;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.POSITION_TARGET_EPSILON;
import static frc.robot.subsystems.superstructure.tongue.TongueConstants.*;

import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;
import java.util.Optional;

public class TongueIOTalonFX extends GenericSuperstructureIOTalonFX implements TongueIO {

  public TongueIOTalonFX() {
    super(
        TONGUE_CONFIG.motorID(),
        INVERT_MOTOR,
        SUPPLY_CURRENT_LIMIT,
        Optional.empty(),
        TONGUE_CONFIG.reduction(),
        UPPER_EXTENSION_LIMIT,
        LOWER_EXTENSION_LIMIT,
        UPPER_VOLT_LIMIT,
        LOWER_VOLT_LIMIT,
        ZEROING_VOLTS,
        ZEROING_OFFSET,
        ZEROING_VOLTAGE_THRESHOLD,
        POSITION_TARGET_EPSILON);
    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        GRAVITY_TYPE);
  }
}
