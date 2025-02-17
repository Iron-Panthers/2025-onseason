package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DigitalInput;

public class RollerSensorsIOComp implements RollerSensorsIO {
  // FIXME; pretty sure rio DIO pullup
  private final DigitalInput intakeSensor = new DigitalInput(7);
  private final DigitalInput topSensor1 = new DigitalInput(8);
  private final DigitalInput topSensor2 = new DigitalInput(9);

  @Override
  public void updateInputs(RollerSensorsIOInputs inputs) {
    inputs.intakeDetected = !intakeSensor.get();
    inputs.pole1Detected = !topSensor1.get();
    inputs.pole2Detected = !topSensor2.get();
  }
}
