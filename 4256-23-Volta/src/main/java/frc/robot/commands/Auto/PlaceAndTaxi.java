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
import frc.robot.commands.Clamp.CloseClamp;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.FormX;
import frc.robot.commands.System.PlaceHigh;
import frc.robot.commands.System.ResetToBottom;
import frc.robot.subsystems.*;

public class PlaceAndTaxi extends SequentialCommandGroup {

  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Gyro gyro = Gyro.getInstance();
  Clamp clamp = Clamp.getInstance();
  Elevator elevator = Elevator.getInstance();
  PIDController xController = new PIDController(2, 0, 0);
  PIDController yController = new PIDController(2,0, 0);
  PIDController thetaController = new PIDController(-2, 0, 0);
  PathPlannerTrajectory autoPath = PathPlanner.loadPath("Place And Taxi", 1, 1);

  PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(
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
  public PlaceAndTaxi() {
    addCommands(
      new InstantCommand(() -> thetaController.enableContinuousInput(0, 360)),
      new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
      new CloseClamp(),
      new PlaceHigh(),
      new InstantCommand(() -> clamp.unclamp()),
      new ResetToBottom(),
      pathCommand,
      new FormX(swerve)
    );
  }
}
