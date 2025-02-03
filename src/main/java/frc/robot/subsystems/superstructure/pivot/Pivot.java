package frc.robot.subsystems.superstructure.pivot;

import frc.robot.subsystems.superstructure.GenericSuperstructure;
// for intaking a note - TODO: ask the intake people what to do, L4 - 45 degrees, for L2 and L3 - 35
// degrees, L1 - TODO: need to test

public class Pivot extends GenericSuperstructure<Pivot.PivotTarget> {
  public enum PivotTarget implements GenericSuperstructure.PositionTarget {
    TOP(90),
    INTAKE(-90),
    STOW(45), // FIXME
    SETUP_L1(0),
    SETUP_L2(90),
    SETUP_L3(90),
    SETUP_L4(45),
    SCORE_L1(0),
    SCORE_L2(20),
    SCORE_L3(20),
    SCORE_L4(0);

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
    setPositionTarget(PivotTarget.TOP);
    setControlMode(ControlMode.STOP);
  }

  
  public PivotTarget nextScorePosition(){
    switch (this.getPositionTarget()) { 
      case SETUP_L1 -> { 
        return PivotTarget.SCORE_L1;
      }
      case SETUP_L2 -> { 
        return PivotTarget.SCORE_L2;
      }
      case SETUP_L3 -> { 
        return PivotTarget.SCORE_L3;
      }
      case SETUP_L4 -> { 
        return PivotTarget.SCORE_L4;
      }
      case SCORE_L1, SCORE_L2, SCORE_L3, SCORE_L4, STOW -> {
        return PivotTarget.STOW;
      }

      // For scoring, these states don't matter
      case TOP -> {
        return PivotTarget.TOP;
      }
      case INTAKE -> {
        return PivotTarget.INTAKE;
      }

      // This should never be reachedS
      default -> {
        return PivotTarget.STOW;
      }
  }
}
}
