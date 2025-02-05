package frc.robot.subsystems.superstructure.tongue;

import frc.robot.subsystems.superstructure.GenericSuperstructure;
// for intaking a note - TODO: ask the intake people what to do, L4 - 45 degrees, for L2 and L3 - 35
// degrees, L1 - TODO: need to test

public class Tongue extends GenericSuperstructure<Tongue.TongueTarget> {
  public enum TongueTarget implements GenericSuperstructure.PositionTarget {
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

  public Tongue(TongueIO io) {
    super("Tongue", io);
    setPositionTarget(TongueTarget.TOP);
    setControlMode(ControlMode.STOP);
  }
}
