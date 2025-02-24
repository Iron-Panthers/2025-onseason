// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.climb.Climb.ClimbTarget;

public class ClimbController extends SubsystemBase {

  private final Climb climb;

  /** Creates a new ClimbController. */
  public ClimbController(Climb climb) {
    this.climb = climb;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climb.periodic();
    climb.setPositionTarget(ClimbTarget.TOP);
  }

  public Command setPositionTargetCommand(ClimbTarget target) {
    return new InstantCommand(
        () -> {
          climb.setPositionTarget(target);
        });
  }

  public boolean climbHitCage() {
    return climb.hitCage();
  }

  public ClimbTarget getClimbTarget() {
    return climb.getPositionTarget();
  }

  public void setClimbTarget(ClimbTarget target) {
    climb.setPositionTarget(target);
  }
}
