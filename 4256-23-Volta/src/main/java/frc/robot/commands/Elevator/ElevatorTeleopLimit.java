// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorTeleopLimit extends CommandBase {

  private Elevator elevator;
  /** Creates a new ElevatorHigh. */
  public ElevatorTeleopLimit(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elevator.setElevatorTeleopLimit();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevator.getElevatorEncoderPosition() >= Constants.ELEVATOR_TELEOP_LIMIT_POSITION) {
      return true;
    } else {
      return false;
    }
  }
}
