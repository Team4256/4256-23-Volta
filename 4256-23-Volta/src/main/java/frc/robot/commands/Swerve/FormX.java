// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class FormX extends CommandBase {
  private final SwerveSubsystem swerveDrive;

  public FormX(SwerveSubsystem swerve) {
    swerveDrive = swerve; 
    addRequirements(swerveDrive); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    swerveDrive.formX();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
