// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToTarget extends CommandBase {
  private final SwerveSubsystem swerveDrive;
  private final Limelight limelight;
  private boolean fieldOrient = true;
  private PIDController orientationPID = new PIDController(-.0242, 0, 0); //Values must be negative

  public MoveToTarget(SwerveSubsystem swerve, Limelight camera) {
    swerveDrive = swerve; // Set the private membeParametersr to the input drivetrain
    limelight = camera;
    addRequirements(swerveDrive); // Because this will be used as a default command, add the subsystem which will
                                   // use this as the default
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = orientationPID.calculate(limelight.getTargetOffsetDegrees(),0);
    double ySpeed = orientationPID.calculate(limelight.getTargetOffsetDegrees(),0);
    double angularSpeed = orientationPID.calculate(limelight.getTargetOffsetDegrees(),0);
    

    swerveDrive.drive(0, 0, angularSpeed, fieldOrient);

      SmartDashboard.putNumber("ALignment Speed", angularSpeed);
      SmartDashboard.putBoolean("Has Target", limelight.hasTarget());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)  {

    SmartDashboard.putBoolean("DrivingByLimelight", true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}