package frc.robot.subsystems.superstructure.pivot;

import frc.robot.subsystems.superstructure.GenericSuperstructure;
// for intaking a note - TODO: ask the intake people what to do, L4 - 45 degrees, for L2 and L3 - 35
// degrees, L1 - TODO: need to test

public class Pivot extends GenericSuperstructure<Pivot.PivotTarget> {
  public enum PivotTarget implements GenericSuperstructure.PositionTarget {
    TOP(0),
    SETUP_L1(0),
    SETUP_L2(0),
    SETUP_L3(0),
    SETUP_L4(0),
    SCORE_L1(0),
    SCORE_L2(0),
    SCORE_L3(0),
    SCORE_L4(0),
    INTAKE(0),
    TEST_N25(-0.27),
    TEST_25(0.25),
    TEST_5(0.45),
    TEST_0(0);
    TOP(90),
    INTAKE(270),
    STOW(270), // FIXME
    L1(285),
    L2(285),
    L3(0),
    L4(0);

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
}
