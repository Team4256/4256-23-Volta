// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.FormX;
import frc.robot.subsystems.*;

public class DirectBalance extends SequentialCommandGroup {

  SwerveSubsystem swerve = new SwerveSubsystem();
  Gyro gyro = Gyro.getInstance();
  Intake intake = Intake.getInstance();
  PIDController xController = new PIDController(2, 0, 0);
  PIDController yController = new PIDController(2, 0, 0);
  PIDController thetaController = new PIDController(-2, 0, 0.0);

  PathPlannerTrajectory autoPath = PathPlanner.loadPath("Over and Back", 1, 1);

  PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      autoPath,
      swerve::getPose,
      Constants.DRIVE_KINEMATICS,
      xController,
      yController,
      thetaController,
      swerve::setModuleStates,
      false,
      swerve);

  /** Creates a new DirectBalance Command. */
  public DirectBalance() {
    addCommands(
        new InstantCommand(() -> gyro.reset()),
        new InstantCommand(() -> thetaController.enableContinuousInput(-180, 180)),
        new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
        new InstantCommand(() -> intake.intakeUp()),
        command,
        new AutoBalance(swerve)

    );
  }
}
