package frc.robot.subsystems.superstructure.climb;

import frc.robot.subsystems.superstructure.GenericSuperstructure;

public class Climb extends GenericSuperstructure<Climb.ClimbTarget> {
  public enum ClimbTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(2.25), // FIXME NOT REAL
    TOP( 10); // FIXME NOT REAL
    private double position = 0;

    private ClimbTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  public Climb(ClimbIO io) {
    super("Climb", io);
    setPositionTarget(ClimbTarget.BOTTOM);
    setControlMode(ControlMode.STOP);
  }
}
