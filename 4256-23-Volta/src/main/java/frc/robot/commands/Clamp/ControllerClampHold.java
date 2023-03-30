// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Clamp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Clamp;

public class ControllerClampHold extends CommandBase {

  private final CommandXboxController controller;
  private final PIDController clampPidController;
  private Clamp clamp;
  private double lastSetpoint;
  /** Creates a new ControllerampHigh. */
  public ControllerClampHold(Clamp clamp, CommandXboxController gunnerController) {
    this.clamp = clamp;
    this.controller = gunnerController;
    this.clampPidController = new PIDController(.05, 0,
        .00);
    addRequirements(clamp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

    if (Math.abs(-controller.getRightY() * 0.5) >= .2) {
      clamp.setClampSpeed(-controller.getRightY() * 0.5);
      lastSetpoint = clamp.getCANCoderAngle();
    } else {
      SmartDashboard.putNumber("last setpoint", lastSetpoint);
      //double speed = clampPidController.calculate(clamp.getCANCoderAngle(), lastSetpoint);
      double speed = clampPidController.calculate(clamp.getCANCoderAngle(), clamp.getCANCoderAngle());

      if (speed > .3) {
        clamp.stop();
      } else {
        clamp.setClampSpeed(clampPidController.calculate(clamp.getCANCoderAngle(), lastSetpoint));

      }
      //clamp.setClampSpeed(clampPidController.calculate(clamp.getCANCoderAngle(), lastSetpoint));
    }


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
