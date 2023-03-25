// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoMoveToTargetHigh extends CommandBase {
  private final SwerveSubsystem swerveDrive;
  private final Limelight limelight;
  private final Gyro gyro = Gyro.getInstance();
  private boolean fieldOrient = true;
  private PIDController orientationPID = new PIDController(-0.03, 0, -0.001); //Values must be negative
  private PIDController positionPID = new PIDController(0.04, 0, -0.0000); //Values must be negative (.02)
  private PIDController forwardPID = new PIDController(1.3, 0, -0.0000); //Values must be negative (.02)

  public AutoMoveToTargetHigh(SwerveSubsystem swerve, Limelight camera) {
    swerveDrive = swerve; // Set the private membeParametersr to the input drivetrain
    limelight = camera;
    addRequirements(swerveDrive); // Because this will be used as a default command, add the subsystem which will
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forwardSpeed = forwardPID.calculate(limelight.getTargetArea(), Constants.LIMELIGHT_TOP_CONE_AREA_THRESHOLD);
    double forwardMoveSpeed = Math.max(-.2, Math.min(forwardSpeed, .2));
    double xSpeed = -positionPID.calculate(limelight.getXOffset(),0);
    double strafeSpeed = Math.max(-.2, Math.min(xSpeed, .2));
    double angularSpeed = -orientationPID.calculate(gyro.getAngle(),0);
    double spinSpeed = Math.max(-.2, Math.min(angularSpeed, .2));

    SmartDashboard.putNumber("forwardSpeed", forwardMoveSpeed);
    
    // if (limelight.getTargetArea() <= Constants.LIMELIGHT_AREA_THRESHOLD) {
    //   swerveDrive.drive(0.2, -strafeSpeed, spinSpeed, fieldOrient);
    // } else {
    //   swerveDrive.drive(0, -strafeSpeed, spinSpeed, fieldOrient);
    // }

    swerveDrive.drive(forwardMoveSpeed, -strafeSpeed, spinSpeed, fieldOrient);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (limelight.getTargetArea() <= Constants.LIMELIGHT_AREA_THRESHOLD) {
    //   return false;
    // } else {
    //   return true;
    // }
    return false;
  }
}
