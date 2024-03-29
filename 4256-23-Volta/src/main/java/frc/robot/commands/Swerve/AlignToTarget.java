// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveSubsystem;


public class AlignToTarget extends CommandBase {
  private final SwerveSubsystem swerveDrive;
  private final Limelight limelight;
  private final CommandXboxController controller;
  private boolean fieldOrient = true;
  private PIDController orientationPID = new PIDController(-0.025, 0, -0.007); //Values must be negative

  public AlignToTarget(SwerveSubsystem swerve, Limelight camera, CommandXboxController driverController) {
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

    double angularSpeed = -orientationPID.calculate(limelight.getXOffset(),0);
    double spinSpeed = Math.max(-.4, Math.min(angularSpeed, .4));
    swerveDrive.drive(-controller.getLeftY(), -controller.getLeftX(), spinSpeed, fieldOrient);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
