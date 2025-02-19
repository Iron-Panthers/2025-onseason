package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import frc.robot.subsystems.superstructure.tongue.Tongue;
import frc.robot.subsystems.superstructure.tongue.Tongue.TongueTarget;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    L4, // Scoring in L4
    L3, // Scoring in L3
    L2, // Scoring in L2
    L1, // Scoring in the trough
    TOP, // Apex
    INTAKE,
    STOW, // Going to the lowest position
    ZERO, // Zero the motor
    CLIMB_UP, //Climb up
    CLIMB_DOWN, //Climb down
    STOP; // Stop the superstructure
  }

  private SuperstructureState currentState = SuperstructureState.ZERO; // current state
  private SuperstructureState targetState = SuperstructureState.ZERO; // current target state

  private final Elevator elevator;
  private final Pivot pivot;
  private final Tongue tongue;
  private final Climb climb;

  public Superstructure(Elevator elevator, Pivot pivot, Tongue tongue, Climb climb) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.tongue = tongue;
    this.climb = climb;
    pivot.setPositionTarget(PivotTarget.TOP);
    elevator.setPositionTarget(ElevatorTarget.BOTTOM);
    tongue.setPositionTarget(TongueTarget.TOP);
  }

  @Override
  public void periodic() {
    switch (targetState) { // switch on the target state
      case L1 -> {
        elevator.setPositionTarget(ElevatorTarget.L1);
        pivot.setPositionTarget(PivotTarget.SETUP_L1);
        tongue.setPositionTarget(TongueTarget.L1);

        // check for state transitions
        if (this.superstructureReachedTarget()) {
          if (targetState == SuperstructureState.L2) {
            this.currentState = SuperstructureState.L2;
          } else if (targetState != currentState) {
            this.currentState = SuperstructureState.STOW;
          }
        }
      }
      case L2 -> {
        elevator.setPositionTarget(ElevatorTarget.L2);
        pivot.setPositionTarget(PivotTarget.SETUP_L2);
        tongue.setPositionTarget(TongueTarget.L2);

        // check for state transitions
        if (this.superstructureReachedTarget()) {
          if (targetState != currentState) {
            this.currentState = SuperstructureState.L1;
          }
        }
      }
      case L3 -> {
        elevator.setPositionTarget(ElevatorTarget.L3);
        pivot.setPositionTarget(PivotTarget.SETUP_L3);
        tongue.setPositionTarget(TongueTarget.L3);

        // check for state transitions
        if (this.superstructureReachedTarget()) {
          if (targetState != currentState) {
            this.currentState = SuperstructureState.L4;
          }
        }
      }
      case L4 -> {
        elevator.setPositionTarget(ElevatorTarget.L4);
        pivot.setPositionTarget(PivotTarget.SETUP_L4);
        tongue.setPositionTarget(TongueTarget.L4);

        // check for state transitions
        if (this.superstructureReachedTarget()) {
          if (targetState == SuperstructureState.L3) {
            this.currentState = SuperstructureState.L3;
          } else if (targetState != currentState) {
            this.currentState = SuperstructureState.TOP;
          }
        }
      }
      case TOP -> {
        elevator.setPositionTarget(ElevatorTarget.BOTTOM);
        pivot.setPositionTarget(PivotTarget.TOP);
        tongue.setPositionTarget(TongueTarget.TOP);

        // check for state transitions
        if (this.superstructureReachedTarget()) {
          if (targetState == SuperstructureState.L4 || targetState == SuperstructureState.L3) {
            this.currentState = SuperstructureState.L4;
          } else if (targetState != currentState) {
            this.currentState = SuperstructureState.STOW;
          }
        }
      }

      case STOW -> {
        elevator.setPositionTarget(ElevatorTarget.BOTTOM);
        pivot.setPositionTarget(PivotTarget.TOP);
        tongue.setPositionTarget(TongueTarget.TOP);

        // check for state transitions
        if (this.superstructureReachedTarget()) {
          if (targetState == SuperstructureState.INTAKE) {
            this.currentState = SuperstructureState.INTAKE;
          } else if (targetState == SuperstructureState.L1
              || targetState == SuperstructureState.L2) {
            this.currentState = SuperstructureState.L1;
          } else if (targetState != currentState) {
            this.currentState = SuperstructureState.TOP;
          }
        }
      }
      case INTAKE -> {
        elevator.setPositionTarget(ElevatorTarget.INTAKE);
        pivot.setPositionTarget(PivotTarget.INTAKE);
        tongue.setPositionTarget(TongueTarget.INTAKE);

        // check for state transitions
        if (this.superstructureReachedTarget()) {
          if (targetState != currentState) {
            this.currentState = SuperstructureState.STOW;
          }
        }
      }
      case ZERO -> {
        elevator.setZeroing(true);
        if (elevator.getFilteredSupplyCurrentAmps()
            > ElevatorConstants
                .ZEROING_VOLTAGE_THRESHOLD) { // check if the elevator is done zeroing and set
          // offsets accordingly
          elevator.setOffset();
          elevator.setControlMode(ControlMode.POSITION);
          elevator.setZeroing(false);

          setTargetState(SuperstructureState.STOW);
        }
      }
      case CLIMB_UP -> {
        // climb up FIXME
      }
      case CLIMB_DOWN -> {
        //climb down FIXME
      }
      case STOP -> {
        elevator.setControlMode(ControlMode.STOP);
        pivot.setControlMode(ControlMode.STOP);
        tongue.setControlMode(Tongue.ControlMode.STOP);
      }
    }

    elevator.periodic();
    pivot.periodic();
    tongue.periodic();

    Logger.recordOutput("Superstructure/TargetState", targetState);
    Logger.recordOutput("Superstructure/Elevator reached target", elevator.reachedTarget());
    Logger.recordOutput("Superstructure/Pivot reached target", pivot.reachedTarget());
    Logger.recordOutput("Superstructure/Reached Target", superstructureReachedTarget());
  }

  // Target state getter and setter
  public void setTargetState(SuperstructureState superstructureState) {
    targetState = superstructureState;
  }

  public SuperstructureState getTargetState() {
    return targetState;
  }

  // go to target state command factory
  public Command goToStateCommand(SuperstructureState superstructureState) {
    return new FunctionalCommand(
        () -> {
          setTargetState(superstructureState);
        },
        () -> {},
        (e) -> {},
        () -> {
          return currentState == targetState && superstructureReachedTarget();
        },
        this);
  }

  /**
   * Get the position of the elevator
   *
   * @return the position of the elevator
   */
  public double getElevatorPosition() {
    return elevator.getPosition();
  }

  /**
   * Get the position of the pivot
   *
   * @return the position of the pivot
   */
  public double getPivotPosition() {
    return pivot.getPosition();
  }

  /**
   * Get the supply current of the elevator
   *
   * @return the supply current of the elevator
   */
  public double getElevatorSupplyCurrentAmps() {
    return elevator.getSupplyCurrentAmps();
  }
  /**
   * Get the supply current of the pivot
   *
   * @return the supply current of the pivot
   */
  public double getPivotSupplyCurrentAmps() {
    return pivot.getSupplyCurrentAmps();
  }

  /**
   * @return a boolean that says weather or not both of our mechanisms have finished zeroing
   */
  public boolean notZeroing() {
    return !elevator.isZeroing();
  }

  /**
   * @return if both subsystems in the superstructure have reached their target
   */
  public boolean superstructureReachedTarget() {
    return elevator.reachedTarget() && pivot.reachedTarget();
  }
}
