package frc.robot.subsystems.superstructure.tongue;

import static frc.robot.subsystems.superstructure.tongue.TongueConstants.*;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Servo;
import org.littletonrobotics.junction.AutoLogOutput;

public class TongueIOServo implements TongueIO {

  Servo tongueIOServo;
  AnalogEncoder tongueSensor;

  public TongueIOServo() {
    tongueIOServo = new Servo(TongueConstants.TONGUE_CONFIG.servoID());
    tongueSensor = new AnalogEncoder(TongueConstants.TONGUE_CONFIG.servoSensorID());
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
    inputs.angle = tongueIOServo.get();
  }

  @AutoLogOutput(key = "Superstructure/Tongue/Position")
  public double getPosition() {
    return tongueSensor.get();
  }
}
