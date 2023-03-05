// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;
import frc.robot.commands.Clamp.CloseClamp;
import frc.robot.commands.Intake.IntakeDown;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.FormX;
import frc.robot.commands.System.PlaceHigh;
import frc.robot.commands.System.ResetToBottom;
import frc.robot.subsystems.*;

public class LeftConePlaceAndGrab extends SequentialCommandGroup {

  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Gyro gyro = Gyro.getInstance();
  Clamp clamp = Clamp.getInstance();
  Elevator elevator = Elevator.getInstance();
  Intake intake = Intake.getInstance();
  PIDController xController = new PIDController(2, 0, 0);
  PIDController yController = new PIDController(2,0, 0);
  PIDController thetaController = new PIDController(-2, 0, 0);
  //PathPlannerTrajectory autoPath = PathPlanner.loadPath("Left Cone Place And Grab", 1, 1);

  PathPlannerTrajectory autoPath1 = PathPlanner.loadPath("Left Cone Place And Grab 1", 1, 1);
  PathPlannerTrajectory autoPath2 = PathPlanner.loadPath("Left Cone Place And Grab 2", 1, 1);
  PathPlannerTrajectory autoPath3 = PathPlanner.loadPath("Left Cone Place And Grab 3", 1, 1);

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

  PPSwerveControllerCommand pathCommand3 = new PPSwerveControllerCommand(
      autoPath3,
      swerve::getPose,
      Constants.DRIVE_KINEMATICS,
      xController,
      yController,
      thetaController,
      swerve::setModuleStates,
      false,
      swerve);

  /** Creates a new DirectBalance Command. */
  public LeftConePlaceAndGrab() {

    HashMap<String, Command> eventMap1 = new HashMap<>();
    eventMap1.put("Clamp", new InstantCommand(() -> clamp.clamp()));
    eventMap1.put("Place High", new PlaceHigh());
    eventMap1.put("Unclamp", new InstantCommand(() -> clamp.unclamp()));
    eventMap1.put("Reset To Bottom", new ResetToBottom());
    eventMap1.put("Intake Down", new InstantCommand(() -> intake.intakeDown()));

    HashMap<String, Command> eventMap2 = new HashMap<>();
    eventMap2.put("Intake", new RunIntake());

    HashMap<String, Command> eventMap3 = new HashMap<>();
    eventMap1.put("Clamp", new InstantCommand(() -> clamp.clamp()));
    eventMap1.put("Place High", new PlaceHigh());
    eventMap1.put("Unclamp", new InstantCommand(() -> clamp.unclamp()));

    FollowPathWithEvents command1 = new FollowPathWithEvents(
    pathCommand1,
    autoPath1.getMarkers(),
    eventMap1
    );

    FollowPathWithEvents command2 = new FollowPathWithEvents(
    pathCommand2,
    autoPath2.getMarkers(),
    eventMap2
    );

    FollowPathWithEvents command3 = new FollowPathWithEvents(
    pathCommand3,
    autoPath3.getMarkers(),
    eventMap3
    );

    addCommands(
      new InstantCommand(() -> gyro.reset()),
      new InstantCommand(() -> thetaController.enableContinuousInput(-180, 180)),
      new InstantCommand(() -> swerve.resetOdometer(autoPath1.getInitialPose())),
      new InstantCommand(() -> clamp.clamp()),
      new PlaceHigh(),
      command1,
      new InstantCommand(() -> clamp.unclamp()),
      new ResetToBottom(),
      new InstantCommand(() -> intake.intakeDown()),
      command2,
      new InstantCommand(() -> clamp.clamp()),
      //new PlaceHigh(),
      command3,
      new InstantCommand(() -> clamp.unclamp())
      //new ResetToBottom()
    );
  }
}
