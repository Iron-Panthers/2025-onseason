package frc.robot.subsystems.superstructure.climb;

import static frc.robot.subsystems.superstructure.climb.ClimbConstants.INDUCTION_PORT_NUMBER;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.superstructure.GenericSuperstructure;

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
   // linear filter for superstrucure
   private final LinearFilter supplyCurrentFilter;
   private double filteredSupplyCurrentAmps = 0;
   //induction sensor
   private DigitalInput inductionSensor;
 
  
  //run tino the cage - sensor triggers - flash leds to tell driver - button presses : reels it in or out
  //CAN'T BACKOUT
  //set position for intake in a cage, a button to climb up or down
  public Climb(ClimbIO io) {
    super("Climb", io);
    inductionSensor = new DigitalInput(INDUCTION_PORT_NUMBER);
    setPositionTarget(ClimbTarget.BOTTOM);
    setControlMode(ControlMode.STOP);
    // setup the linear filter
    supplyCurrentFilter = LinearFilter.movingAverage(30);
  }
  //checks if the sensor has hit the cage
  public boolean hitCage() {
    return inductionSensor.get(); 
  }
  private boolean zeroing = false;
  @Override
  public void periodic() {

    super.periodic();

    // for zeroing
    // calculate our new filtered supply current for the elevator
    filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());
    if (zeroing) {
      superstructureIO.runCharacterization();
    }
    Logger.recordOutput(
        "Superstructure/" + name + "/Filtered supply current amps", getFilteredSupplyCurrentAmps());
  }

  public double getFilteredSupplyCurrentAmps() {
    return filteredSupplyCurrentAmps;
  }

  public void setZeroing(boolean zeroing) {
    this.zeroing = zeroing;
  }

  public boolean isZeroing() {
    return zeroing;
  }
}
