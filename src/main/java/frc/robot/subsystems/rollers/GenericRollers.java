package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

public abstract class GenericRollers<G extends GenericRollers.VoltageTarget> {
  public interface VoltageTarget {
    double getVolts();
  }

  private boolean beamBreakBroken;

  private final String name;
  private final GenericRollersIO rollerIO;
  private GenericRollersIOInputsAutoLogged inputs = new GenericRollersIOInputsAutoLogged();

  private G voltageTarget;

  public GenericRollers(String name, GenericRollersIO rollerIO) {
    this.name = name;
    this.rollerIO = rollerIO;
  }

  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    rollerIO.runVolts(voltageTarget.getVolts());
    Logger.recordOutput("Rollers/" + name + "/Target", voltageTarget.toString());

    beamBreakBroken = inputs.beamBreakBroken;
    Logger.recordOutput("Rollers/" + name + "/BeamBreakBroken", beamBreakBroken);
  }

  public G getVoltageTarget() {
    return voltageTarget;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public boolean getBeamBreakBroken() {
    return beamBreakBroken;
  }

  public void setVoltageTarget(G voltageTarget) {
    this.voltageTarget = voltageTarget;
  }
}
