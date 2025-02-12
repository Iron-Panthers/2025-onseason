package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.subsystems.superstructure.GenericSuperstructure;

public class Elevator extends GenericSuperstructure<Elevator.ElevatorTarget> {
  public enum ElevatorTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(5.25), // 25 and 7.25, made it a bit bigger
    L1(5.5), // FIXME: 26 and 21.5
    L2(7.25), // 24 and 53.75
    L3(21.5), // 0 and 53.75
    L4(53.75),
    SOURCE(20),
    SETUP_INTAKE(22),
    INTAKE(16.5); // FIXME
    private double position = 0;

    private ElevatorTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  // linear filter for superstrucure
  private final LinearFilter supplyCurrentFilter;
  private double filteredSupplyCurrentAmps = 0;

  private boolean zeroing = false;

  public Elevator(ElevatorIO io) {
    super("Elevator", io);
    setPositionTarget(ElevatorTarget.BOTTOM);
    setControlMode(ControlMode.STOP);

    // setup the linear filter
    supplyCurrentFilter = LinearFilter.movingAverage(30);
  }
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

  public boolean aboveSafeHeightForPivot() {
    return this.getPosition() > ElevatorConstants.MIN_SAFE_HEIGHT_FOR_PIVOT;
  }

  public void setZeroing(boolean zeroing) {
    this.zeroing = zeroing;
  }

  public boolean isZeroing() {
    return zeroing;
  }

}
