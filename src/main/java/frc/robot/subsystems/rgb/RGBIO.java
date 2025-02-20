package frc.robot.subsystems.rgb;


import frc.robot.subsystems.rgb.RGBConstants.RGBMessage;

import org.littletonrobotics.junction.AutoLog;

public interface RGBIO {
  @AutoLog
  class RGBIOInputs {}

  default void updateInputs(RGBIOInputs inputs) {}

  default void displayMessage(RGBMessage lightMessage) {}

  default void clear() {}
}
