// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Limelight;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Xbox;

public class MoveToTarget extends CommandBase {
  private final SwerveSubsystem swerveDrive;
  private final Limelight limelight;
  private final CommandXboxController controller;
  private final Gyro gyro = Gyro.getInstance();
  private boolean fieldOrient = true;
  private PIDController orientationPID = new PIDController(-0.01, 0, -0.00); //Values must be negative
  private PIDController positionPID = new PIDController(-0.038, 0, -0.0); //Values must be negative (.02)

  public MoveToTarget(SwerveSubsystem swerve, Limelight camera, CommandXboxController driverController) {
    swerveDrive = swerve; // Set the private membeParametersr to the input drivetrain
    limelight = camera;
    this.controller = driverController;
    addRequirements(swerveDrive); // Because this will be used as a default command, add the subsystem which will
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -positionPID.calculate(limelight.getXOffset(),0);
    double strafeSpeed = Math.max(-.4, Math.min(xSpeed, .4));
    double angularSpeed = -orientationPID.calculate(gyro.getAngle(),0);
    double spinSpeed = Math.max(-.4, Math.min(angularSpeed, .4));


    swerveDrive.drive(-controller.getLeftY(), strafeSpeed, angularSpeed, fieldOrient);

      // SmartDashboard.putNumber("Limeight X Error", limelight.getXOffset());
      // SmartDashboard.putNumber("Alignment Speed", spinSpeed);
      // SmartDashboard.putNumber("Auto Strafe Speed", strafeSpeed);
      // SmartDashboard.putBoolean("Has Target", limelight.hasTarget());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)  {

    //SmartDashboard.putBoolean("DrivingByLimelight", true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
