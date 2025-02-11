package frc.robot.subsystems.superstructure.tongue;

import org.littletonrobotics.junction.AutoLog;

public interface TongueIO {
  @AutoLog
  class TongueIOInputs {
    public boolean connected = true;
    public double positionRotations = 0;
    public double velocityRotPerSec = 0;
    public double appliedVolts = 0;
    public double tempCelsius = 0;
  }

  default void updateInputs(TongueIOInputsAutoLogged inputs) {}

  default void runPosition(double position) {}

  default void stop() {}
}
