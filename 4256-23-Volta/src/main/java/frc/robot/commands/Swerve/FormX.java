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

public class FormX extends CommandBase {
  private final SwerveSubsystem swerveDrive;
  private final Gyro gyro = Gyro.getInstance();
  private boolean fieldOrient = true;
  private PIDController orientationPID = new PIDController(-0.025, 0, -0.007); //Values must be negative
  

  public FormX(SwerveSubsystem swerve) {
    swerveDrive = swerve; // Set the private membeParametersr to the input drivetrain
    addRequirements(swerveDrive); // Because this will be used as a default command, add the subsystem which will
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      swerveDrive.formX();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)  {
    //swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
