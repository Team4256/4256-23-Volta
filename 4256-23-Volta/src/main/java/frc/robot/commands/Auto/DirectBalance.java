// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.*;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

public class DirectBalance extends SequentialCommandGroup {

  //SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  SwerveSubsystem swerve = new SwerveSubsystem();
  Gyro gyro = Gyro.getInstance();
    PIDController xController = new PIDController(3, 0, 0);
    PIDController yController = new PIDController(3, 0, 0);
    PIDController thetaController = new PIDController(-1, 0, 0.0);
    
  PathPlannerTrajectory autoPath = PathPlanner.loadPath("Direct Balance", 1, .1);

  PPSwerveControllerCommand command = new PPSwerveControllerCommand(
    autoPath,
    swerve::getPose,
    Constants.DRIVE_KINEMATICS,
    xController,
    yController,
    thetaController,
    swerve::setModuleStates,
    false,
    swerve
  );

  /** Creates a new ThreeBallAutoBottom. */
  public DirectBalance() { 
    addCommands(
      new InstantCommand(() -> gyro.reset()),
      new InstantCommand(() -> thetaController.enableContinuousInput(0, 360)),
      new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
      command,
      new InstantCommand(() -> swerve.stopModules())
      
    );
  }
}
