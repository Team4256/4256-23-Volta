// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Swerve.AlignToTarget;
import frc.robot.commands.Swerve.AlignToZero;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.BlankCommand;
import frc.robot.commands.Swerve.ControllerDrive;
import frc.robot.commands.Swerve.FormX;
import frc.robot.commands.Swerve.MoveToTarget;
import frc.robot.commands.Auto.DirectBalance;
import frc.robot.commands.Auto.TwoConeAutoTop;
import frc.robot.commands.Clamp.SetClampLow;
import frc.robot.commands.Clamp.SetClampTop;
import frc.robot.commands.Clamp.SetClampMid;
import frc.robot.commands.Clamp.SetClampGrab;
import frc.robot.commands.Clamp.CloseClamp;
import frc.robot.commands.Clamp.ControllerClamp;
import frc.robot.commands.Clamp.OpenClamp;
import frc.robot.commands.Elevator.IncrementElevator;
import frc.robot.commands.Intake.IntakeDown;
import frc.robot.commands.Intake.IntakeUp;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.RunIntakeReverse;
import frc.robot.commands.Elevator.ControllerElevator;
import frc.robot.commands.Elevator.DecrementElevator;
import frc.robot.commands.Elevator.ElevatorBottom;
import frc.robot.commands.Elevator.TiltElevatorDown;
import frc.robot.commands.Elevator.ElevatorHigh;
import frc.robot.commands.Elevator.ElevatorLow;
import frc.robot.commands.Elevator.ElevatorMid;
import frc.robot.commands.Elevator.TiltElevatorUp;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_ID);
  CommandXboxController gunnerController = new CommandXboxController(Constants.GUNNER_CONTROLLER_ID);
  private final SwerveSubsystem robotDrive = new SwerveSubsystem();
  private final Elevator elevator = Elevator.getInstance();
  private final Clamp clamp = Clamp.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Limelight camera = new Limelight();
  private final Gyro gyro = Gyro.getInstance();

  // Intake
  private final Command intakeDown = new IntakeDown();
  private final Command intakeUp = new IntakeUp();
  private final Command runIntake = new RunIntake();
  private final Command runIntakeReverse = new RunIntakeReverse();

  // Elevator
  private final Command tiltElevatorUp = new TiltElevatorUp(elevator);
  private final Command tiltElevatorDown = new TiltElevatorDown(elevator);
  private final Command elevatorHigh = new ElevatorHigh(elevator);
  private final Command elevatorMid = new ElevatorMid(elevator);
  private final Command elevatorLow = new ElevatorLow(elevator);
  private final Command elevatorBottom = new ElevatorBottom(elevator);
  private final Command incrementElevator = new IncrementElevator(elevator);
  private final Command controllerElevator = new ControllerElevator(elevator, gunnerController);

  // Clamp
  private final Command setClampTop = new SetClampTop(clamp);
  private final Command setClampMid = new SetClampMid(clamp);
  private final Command setClampLow = new SetClampLow(clamp);
  private final Command setClampGrab = new SetClampGrab(clamp);
  private final Command controllerClamp = new ControllerClamp(clamp, gunnerController);
  private final Command openClamp = new OpenClamp();
  private final Command closeClamp = new CloseClamp();

  // Swerve
  private final ControllerDrive controllerDrive = new ControllerDrive(robotDrive, driverController);
  private final Command alignToTarget = new AlignToTarget(robotDrive, camera, driverController);
  private final Command alignToZero = new AlignToZero(robotDrive, driverController);
  private final Command moveToTarget = new MoveToTarget(robotDrive, camera, driverController);
  private final Command autoBalance = new AutoBalance(robotDrive);
  private final Command formX = new FormX(robotDrive);
  private final Command twoConeAutoTop = new TwoConeAutoTop();
  private final Command directBalance = new DirectBalance();
  private final Command blankCommand = new BlankCommand(robotDrive);

  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    elevator.setDefaultCommand(controllerElevator);
    clamp.setDefaultCommand(controllerClamp);

    chooser.setDefaultOption("Direct Balance", directBalance);
    chooser.addOption("Two Cone Auto Top", twoConeAutoTop);

    // Put the chooser on the dashboard
    // Shuffleboard.getTab("Competition").add(chooser);
    SmartDashboard.putData(chooser);
  }

  public void setTeleopDefaultCommands() {
    robotDrive.setDefaultCommand(controllerDrive);
  }

  public void setAutoDefaultCommands() {
    robotDrive.setDefaultCommand(blankCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Driver Button Bindings
    driverController.y().onTrue(intakeUp);
    driverController.a().onTrue(intakeDown);
    driverController.b().onTrue(new InstantCommand(() -> gyro.reset()));
    driverController.x().onTrue(formX);
    driverController.start().whileTrue(moveToTarget);
    driverController.back().onTrue(new InstantCommand(() -> elevator.resetElevatorEncoder()));
    driverController.leftBumper().whileTrue(runIntake);
    driverController.rightBumper().whileTrue(runIntakeReverse);

    // Gunner Button Bindings
    gunnerController.y().whileTrue(elevatorHigh);
    gunnerController.a().whileTrue(elevatorBottom);
    gunnerController.b().onTrue(new InstantCommand(() -> clamp.resetClampEncoder()));
    gunnerController.x().whileTrue(elevatorMid);
    gunnerController.start().whileTrue(openClamp);
    gunnerController.back().whileTrue(closeClamp);
    gunnerController.leftBumper().whileTrue(tiltElevatorUp);
    gunnerController.rightBumper().whileTrue(tiltElevatorDown);
    gunnerController.povDown().whileTrue(setClampLow);
    gunnerController.povUp().whileTrue(setClampTop);
    gunnerController.povLeft().whileTrue(setClampMid);
    gunnerController.povRight().whileTrue(setClampGrab);    

  }

  // /**
  // * Use this to pass the autonomous command to the main {@link Robot} class.
  // *
  // * @return the command to run in autonomous
  // */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
