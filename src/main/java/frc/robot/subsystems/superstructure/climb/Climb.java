package frc.robot.subsystems.superstructure.climb;

import frc.robot.subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.superstructure.tongue.Tongue.TongueTarget;
import frc.robot.subsystems.superstructure.tongue.TongueIO;
import frc.robot.subsystems.superstructure.tongue.TongueIOInputsAutoLogged;

public class Climb extends GenericSuperstructure<Climb.ClimbTarget> {
  public enum ClimbTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(2.25), // FIXME: Just a placeholder value
    TOP( 10); // FIXME: Just a placeholder value
    private double position = 0;

    private ClimbTarget(double position) {
      this.position = position;
    }
    public double getPosition() {
   
      return position;
    }
  }
  //run tino the cage - sensor triggers - flash leds to tell driver - button presses : reels it in or out
  // CAN'T BACKOUT
  //set position for intake in a cage, a button to climb up or down
  public Climb(ClimbIO io) {
    super("Climb", io);
    setPositionTarget(ClimbTarget.BOTTOM);
    setControlMode(ControlMode.STOP);
  }

  //FIXME: make a boolean function, that returns whether we're touching cage or not

}
