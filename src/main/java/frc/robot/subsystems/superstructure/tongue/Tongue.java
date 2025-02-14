package frc.robot.subsystems.superstructure.tongue;

import frc.robot.subsystems.superstructure.GenericSuperstructure.ControlMode;
// for intaking a note - TODO: ask the intake people what to do, L4 - 45 degrees, for L2 and L3 - 35
// degrees, L1 - TODO: need to test
import org.littletonrobotics.junction.Logger;

public class Tongue {
  public enum TongueTarget {
    TOP(0),
    L1(0),
    L2(0),
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
    ZERO,
    STOP,
  }

  private ControlMode controlMode = ControlMode.STOP;

  private final TongueIO io;

  private TongueIOInputsAutoLogged inputs = new TongueIOInputsAutoLogged(); // FIXME
  private TongueTarget positionTarget;

  public Tongue(TongueIO io) {
    this.io = io;

    setPositionTarget(TongueTarget.TOP);
    setControlMode(ControlMode.STOP);
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Tongue", inputs);

    // Process control mode
    switch (controlMode) {
      case POSITION -> {
        io.runPosition(positionTarget.getPosition());
      }
      case STOP -> {
        io.stop();
      }
    }

    Logger.recordOutput("Superstructure/" + "Tongue" + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + "Tongue" + "/Control Mode", controlMode.toString());
    Logger.recordOutput("Superstructure/" + "Tongue" + "/Reached target", reachedTarget());
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
    return inputs.positionRotations;
  }

  public boolean reachedTarget() {
    return Math.abs(inputs.positionRotations - positionTarget.getPosition())
        <= TongueConstants.POSITION_TARGET_EPSILON;
  }
}
