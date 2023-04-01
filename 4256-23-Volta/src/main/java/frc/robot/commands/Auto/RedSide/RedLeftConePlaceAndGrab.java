// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.RedSide;

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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.*;
import frc.robot.commands.Clamp.CloseClamp;
import frc.robot.commands.Clamp.SetClampCube;
import frc.robot.commands.Clamp.SpitClamp;
import frc.robot.commands.Clamp.SuckClamp;
import frc.robot.commands.Intake.IntakeDown;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.AutoMoveToTargetHigh;
import frc.robot.commands.Swerve.FormX;
import frc.robot.commands.System.PlaceHigh;
import frc.robot.commands.System.ResetToBottom;
import frc.robot.subsystems.*;

public class RedLeftConePlaceAndGrab extends SequentialCommandGroup {

  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  Gyro gyro = Gyro.getInstance();
  Clamp clamp = Clamp.getInstance();
  Elevator elevator = Elevator.getInstance();
  Intake intake = Intake.getInstance();
  Limelight limelight = Limelight.getInstance();
  PIDController xController = new PIDController(3, 0, 0);
  PIDController yController = new PIDController(3,0, 0);
  PIDController thetaController = new PIDController(-4, 0, 0);
  //PathPlannerTrajectory autoPath = PathPlanner.loadPath("Left Cone Place And Grab", 1, 1);

  PathPlannerTrajectory autoPath = PathPlanner.loadPath("Red Left Cone Place And Grab", 1.5, 1.5);

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
  public RedLeftConePlaceAndGrab() {
    addCommands(
      new InstantCommand(() -> limelight.setPipeline(1)),
      new InstantCommand(() -> gyro.reset()),
      new InstantCommand(() -> thetaController.enableContinuousInput(-180, 180)),
      new InstantCommand(() -> intake.intakeDown()),
      new PlaceHigh(),
      new ParallelRaceGroup(new AutoMoveToTargetHigh(swerve, limelight), new WaitCommand(2.5)),
      new InstantCommand(() -> swerve.formX()),
      new InstantCommand(() -> clamp.unclamp()),
      new ResetToBottom(),
      new SetClampCube(clamp),
      new InstantCommand(() -> swerve.resetOdometer(autoPath.getInitialPose())),
      new ParallelDeadlineGroup(pathCommand, new RunIntake(), new SuckClamp())
      //new ParallelDeadlineGroup(new WaitCommand(1), new RunIntake(), new SuckClamp())
      // new PlaceHigh(),
      // pathCommand3,
      // new ParallelDeadlineGroup(new WaitCommand(.2), new SpitClamp()),
      // new ResetToBottom()
    );
  }
}
