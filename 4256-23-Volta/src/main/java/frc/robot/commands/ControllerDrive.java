// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ControllerDrive extends CommandBase {
  private final SwerveSubsystem swerveDrive;
  private final XboxController controller;
  private boolean fieldOrient = true;



  public ControllerDrive(SwerveSubsystem swerve, XboxController controller) {
    swerveDrive = swerve; // Set the private membeParametersr to the input drivetrain
    this.controller = controller; // Set the private member to the input controller
    addRequirements(swerveDrive); // Because this will be used as a default command, add the subsystem which will
                                   // use this as the default
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //swerveDrive.drive(-controller.getLeftY(), -controller.getLeftX(), -controller.getRightX(), fieldOrient);

    swerveDrive.drive(-controller.getLeftY(), -controller.getLeftX(), -controller.getRightX(), fieldOrient);
      SmartDashboard.putBoolean("Controller Drive", true);
      SmartDashboard.putNumber("Controller Y Value", controller.getLeftX());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    SmartDashboard.putBoolean("DrivingByController", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
