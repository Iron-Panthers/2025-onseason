package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollers;

public class Intake extends GenericRollers<Intake.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(2),
    HOLD(0),
    EJECT(-0.7);

    private double volts;

    private Target(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return volts;
    }
  }

  public Intake(IntakeIO intakeIO) {
    super("Intake", intakeIO);
  }
}
