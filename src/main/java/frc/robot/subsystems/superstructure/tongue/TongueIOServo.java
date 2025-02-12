package frc.robot.subsystems.superstructure.tongue;

import static frc.robot.subsystems.superstructure.tongue.TongueConstants.*;

import edu.wpi.first.wpilibj.Servo;

public class TongueIOServo implements TongueIO {

  Servo tongueIOServo;

  public TongueIOServo() {
    tongueIOServo = new Servo(7);
    // tongueIOServo.setZeroLatch();
    // tongueIOServo.

    // set servo settings
  }

  @Override
  public void runPosition(double position) {
    tongueIOServo.setAngle(position);
  }

  @Override
  public void stop() {
    tongueIOServo.setDisabled();
  }

  @Override
  public void updateInputs(TongueIOInputsAutoLogged inputs) {
    inputs.angle = tongueIOServo.getAngle();
  }
}
