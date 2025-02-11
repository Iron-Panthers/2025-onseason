package frc.robot.subsystems.superstructure.tongue;

import frc.robot.subsystems.superstructure.GenericSuperstructure.ControlMode;
// for intaking a note - TODO: ask the intake people what to do, L4 - 45 degrees, for L2 and L3 - 35
// degrees, L1 - TODO: need to test
import org.littletonrobotics.junction.Logger;

public class Tongue {
  public enum TongueTarget {
    TOP(90),
    SETUP_L1(0),
    SETUP_L2(90),
    SETUP_L3(90),
    SETUP_L4(45),
    SCORE_L1(0),
    SCORE_L2(20),
    SCORE_L3(20),
    SCORE_L4(0),
    INTAKE(-90);

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

  private final String name;
  private final TongueIO io;

  private TongueIOInputsAutoLogged inputs = new TongueIOInputsAutoLogged(); // FIXME
  private TongueTarget positionTarget;

  public Tongue(TongueIO io, String name) {
    this.name = name;
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
        io.runPosition(positionTarget.getPosition());
      }
      case STOP -> {
        io.stop();
      }
    }

    Logger.recordOutput("Superstructure/" + name + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + name + "/Control Mode", controlMode.toString());
    Logger.recordOutput("Superstructure/" + name + "/Reached target", reachedTarget());
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
