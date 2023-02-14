// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  private final SwerveSubsystem swerveDrive;
  private final Gyro gyro = Gyro.getInstance();
  private boolean fieldOrient = true;
  private PIDController balancePID = new PIDController(-0.01, 0, -0.00); // Values must be negative
  private double xSpeed;

  public AutoBalance(SwerveSubsystem swerve) {
    swerveDrive = swerve; // Set the private membeParametersr to the input drivetrain
    balancePID.setTolerance(0.5);
    addRequirements(swerveDrive); // Because this will be used as a default command, add the subsystem which will
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    xSpeed = -Math.min(balancePID.calculate(gyro.getPitch(), 0), 1);

    // Limit the max power
    if (Math.abs(xSpeed) > 0.4) {
      xSpeed = Math.copySign(0.4, xSpeed);
    }
    swerveDrive.drive(-xSpeed, 0, 0, fieldOrient);

    SmartDashboard.putNumber("Balance Speed", xSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
