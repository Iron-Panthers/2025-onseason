package frc.robot.subsystems.superstructure.pivot;

import frc.robot.subsystems.superstructure.GenericSuperstructure;
// for intaking a note - TODO: ask the intake people what to do, L4 - 45 degrees, for L2 and L3 - 35
// degrees, L1 - TODO: need to test

public class Pivot extends GenericSuperstructure<Pivot.PivotTarget> {
  public enum PivotTarget implements GenericSuperstructure.PositionTarget {
    TOP(-82),
    INTAKE(-97),
    STOW(-97), // FIXME
    L1(-110),
    L2(-110),
    L3(130.68),
    SETUP_L4(140),
    SCORE_L4(146);

    private double position;

    private PivotTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  public Pivot(PivotIO io) {
    super("Pivot", io);
    setPositionTarget(PivotTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  /**
   * This function returns weather or not the subsystem has reached its position target
   *
   * @return weather the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(super.getPosition() - (super.getPositionTarget().getPosition() / 360d))
        <= superstructureIO.getPositionTargetEpsilon();
  }
}
