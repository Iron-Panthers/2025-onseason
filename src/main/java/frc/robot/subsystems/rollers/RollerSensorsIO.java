package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
  @AutoLog
  class RollerSensorsIOInputs {
    public boolean intakeDetected = false;
    public boolean pole1Detected = false;
    public boolean pole2Detected = false;
  }

  default void updateInputs(RollerSensorsIOInputs inputs) {}
}
