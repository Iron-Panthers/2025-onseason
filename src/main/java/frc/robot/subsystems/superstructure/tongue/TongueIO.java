package frc.robot.subsystems.superstructure.tongue;

import org.littletonrobotics.junction.AutoLog;

public interface TongueIO {
  @AutoLog
  class TongueIOInputs {
    public double angle = 0;
  }

  default void updateInputs(TongueIOInputsAutoLogged inputs) {}

  default void runPosition(double position) {}

  default void stop() {}
}
