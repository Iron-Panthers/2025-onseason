package frc.robot.subsystems.superstructure.tongue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Tongue extends SubsystemBase {
  public enum TongueTarget {
    TOP(0),
    L1(30),
    L2(30),
    L3(90),
    L4(90),
    INTAKE(0);

    private double position;

    private TongueTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  public enum ControlMode {
    POSITION,
    STOP,
  }

  private ControlMode controlMode = ControlMode.STOP;

  private final String name;
  private final TongueIO io;

  private TongueIOInputsAutoLogged inputs = new TongueIOInputsAutoLogged(); // FIXME
  private TongueTarget positionTarget;

  public Tongue(TongueIO io) {
    this.name = "Tongue";
    this.io = io;

    setPositionTarget(TongueTarget.TOP);
    setControlMode(ControlMode.STOP);
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Process control mode
    switch (controlMode) {
      case POSITION -> {
        io.runPosition(positionTarget.getPosition() + offset);
      }
      case STOP -> {
        io.stop();
      }
    }

    Logger.recordOutput("Tongue/Target", positionTarget.toString());
    Logger.recordOutput("Tongue/Control Mode", controlMode.toString());
    Logger.recordOutput("Tongue/Reached target", reachedTarget());
  }

  public TongueTarget getPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(TongueTarget positionTarget) {
    setControlMode(ControlMode.POSITION);
    this.positionTarget = positionTarget;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }

  public double position() {
    return inputs.angle;
  }

  public boolean reachedTarget() {
    return Math.abs(inputs.angle - positionTarget.getPosition())
        <= TongueConstants.POSITION_TARGET_EPSILON;
  }
}
