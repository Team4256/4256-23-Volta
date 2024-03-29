// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  private final SwerveSubsystem swerveDrive;
  private final Gyro gyro = Gyro.getInstance();
  private boolean fieldOrient = true;
  private PIDController balancePID = new PIDController(-0.008, 0, -0.00); // Values must be negative
  private double ySpeed;

  public AutoBalance(SwerveSubsystem swerve) {
    swerveDrive = swerve; // Set the private membeParametersr to the input drivetrain
    balancePID.setTolerance(8);
    addRequirements(swerveDrive); // Because this will be used as a default command, add the subsystem which will
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    swerveDrive.setBrakeMode();
    ySpeed = -Math.min(balancePID.calculate(gyro.getPitch(), 0), 1);

    // Limit the max power
    if (Math.abs(ySpeed) > 0.2) {
      ySpeed = Math.copySign(0.2, ySpeed);
    }

    if (Math.abs(gyro.getPitch()) < 10) {
      swerveDrive.formX();
    } else {
      swerveDrive.drive(-ySpeed, 0, 0, fieldOrient);
    }
    
    SmartDashboard.putNumber("Balance Speed", ySpeed);
    
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
