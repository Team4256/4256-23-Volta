// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Clamp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Clamp;

public class SuckClamp extends CommandBase {
  private Clamp clamp = Clamp.getInstance();
  /** Creates a new OpenIntake. */
  public SuckClamp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clamp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clamp.suck();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clamp.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
