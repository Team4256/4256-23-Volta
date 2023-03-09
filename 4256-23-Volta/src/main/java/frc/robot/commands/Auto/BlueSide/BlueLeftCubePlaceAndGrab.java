// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.BlueSide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.*;
import frc.robot.commands.Clamp.CloseClamp;
import frc.robot.commands.Clamp.SetClampGrab;
import frc.robot.commands.Clamp.SpitClamp;
import frc.robot.commands.Clamp.SuckClamp;
import frc.robot.commands.Intake.IntakeDown;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.FormX;
import frc.robot.commands.System.PlaceHigh;
import frc.robot.commands.System.ResetToBottom;
import frc.robot.subsystems.*;

public class BlueLeftCubePlaceAndGrab extends SequentialCommandGroup {

  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Gyro gyro = Gyro.getInstance();
  Clamp clamp = Clamp.getInstance();
  Elevator elevator = Elevator.getInstance();
  Intake intake = Intake.getInstance();
  PIDController xController = new PIDController(3, 0, 0);
  PIDController yController = new PIDController(3,0, 0);
  PIDController thetaController = new PIDController(-3, 0, 0);
  //PathPlannerTrajectory autoPath = PathPlanner.loadPath("Left Cone Place And Grab", 1, 1);

  PathPlannerTrajectory autoPath1 = PathPlanner.loadPath("Blue Left Cube Place And Grab", 1, 1);
  // PathPlannerTrajectory autoPath2 = PathPlanner.loadPath("Blue Right Cone Place And Grab 2", 1, 1);
  // PathPlannerTrajectory autoPath3 = PathPlanner.loadPath("Blue Right Cone Place And Grab 3", 1, 1);

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

  // PPSwerveControllerCommand pathCommand2 = new PPSwerveControllerCommand(
  //     autoPath2,
  //     swerve::getPose,
  //     Constants.DRIVE_KINEMATICS,
  //     xController,
  //     yController,
  //     thetaController,
  //     swerve::setModuleStates,
  //     false,
  //     swerve);

  // PPSwerveControllerCommand pathCommand3 = new PPSwerveControllerCommand(
  //     autoPath3,
  //     swerve::getPose,
  //     Constants.DRIVE_KINEMATICS,
  //     xController,
  //     yController,
  //     thetaController,
  //     swerve::setModuleStates,
  //     false,
  //     swerve);

  /** Creates a new DirectBalance Command. */
  public BlueLeftCubePlaceAndGrab() {

    addCommands(
      new InstantCommand(() -> gyro.reset()),
      new InstantCommand(() -> thetaController.enableContinuousInput(-180, 180)),
      new InstantCommand(() -> swerve.resetOdometer(autoPath1.getInitialPose())),
      new InstantCommand(() -> intake.intakeDown()),
      new ParallelDeadlineGroup(new WaitCommand(.5), new SuckClamp()),
      new PlaceHigh(),
      new ParallelDeadlineGroup(new WaitCommand(.2), new SpitClamp()),
      new ResetToBottom(),
      new ParallelDeadlineGroup(pathCommand1, new RunIntake()),
      new SetClampGrab(clamp),
      new ParallelDeadlineGroup(new WaitCommand(1), new RunIntake(), new SuckClamp()),
      new PlaceHigh(),
      new ParallelDeadlineGroup(new WaitCommand(.2), new SpitClamp()),
      new ResetToBottom()
    );
  }
}
