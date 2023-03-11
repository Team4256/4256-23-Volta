// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.RedSide;

import java.time.Instant;

import javax.swing.text.html.FormView;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.*;
import frc.robot.commands.Clamp.CloseClamp;
import frc.robot.commands.Clamp.SpitClamp;
import frc.robot.commands.Clamp.SuckClamp;
import frc.robot.commands.Elevator.ElevatorSmallRaise;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.FormX;
import frc.robot.commands.System.PlaceHigh;
import frc.robot.commands.System.ResetToBottom;
import frc.robot.subsystems.*;

public class RedPlaceAndBalance extends SequentialCommandGroup {

  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Gyro gyro = Gyro.getInstance();
  Clamp clamp = Clamp.getInstance();
  Elevator elevator = Elevator.getInstance();
  Intake intake = Intake.getInstance();
  PIDController xController = new PIDController(3, 0, 0);
  PIDController yController = new PIDController(3, 0, 0);
  PIDController thetaController = new PIDController(-4, 0, 0);

  PathPlannerTrajectory autoPath1 = PathPlanner.loadPath("Red Place And Balance 1", 1, 1);
  PathPlannerTrajectory autoPath2 = PathPlanner.loadPath("Red Place And Balance 2", 1.5, 1.5);

  PPSwerveControllerCommand pathCommand1 = new PPSwerveControllerCommand(
      autoPath1,
      swerve::getPose,
      Constants.DRIVE_KINEMATICS,
      xController,
      yController,
      thetaController,
      swerve::setModuleStates,
      false,
      swerve);
      
PPSwerveControllerCommand pathCommand2 = new PPSwerveControllerCommand(
      autoPath2,
      swerve::getPose,
      Constants.DRIVE_KINEMATICS,
      xController,
      yController,
      thetaController,
      swerve::setModuleStates,
      false,
      swerve);
  /** Creates a new TwoConeAutoTop Command. */
  public RedPlaceAndBalance() {
    addCommands(
        new InstantCommand(() -> thetaController.enableContinuousInput(-180, 180)),
        new InstantCommand(() -> swerve.resetOdometer(autoPath1.getInitialPose())),
        new InstantCommand(() -> intake.intakeDown()),
        new ParallelDeadlineGroup(new WaitCommand(.5), new SuckClamp()),
        new PlaceHigh(),
        pathCommand1,
        new ParallelDeadlineGroup(new WaitCommand(.5), new SpitClamp()),
        new ResetToBottom(),
        new InstantCommand(() -> intake.intakeUp()),
        new WaitCommand(.7),
        new ElevatorSmallRaise(elevator),
        pathCommand2,
        new AutoBalance(swerve)
    );
  }
}