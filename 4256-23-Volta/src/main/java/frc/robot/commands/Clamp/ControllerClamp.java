// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Clamp;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.Xbox;

public class ControllerClamp extends CommandBase {

  private final CommandXboxController controller;

  private Clamp clamp;
  /** Creates a new ControllerampHigh. */
  public ControllerClamp(Clamp clamp, CommandXboxController gunnerController) {
    this.clamp = clamp;
    this.controller = gunnerController;
    addRequirements(clamp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    clamp.setClampSpeed(-controller.getRightY());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clamp.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
