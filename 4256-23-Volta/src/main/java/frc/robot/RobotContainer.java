// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Swerve.AlignToTarget;
import frc.robot.commands.Swerve.AlignToZero;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.AutoMoveToTarget;
import frc.robot.commands.Swerve.BlankCommand;
import frc.robot.commands.Swerve.ControllerDrive;
import frc.robot.commands.Swerve.FormX;
import frc.robot.commands.Swerve.MoveToTarget;
import frc.robot.commands.System.ConeMidPosition;
import frc.robot.commands.System.FeederStationPosition;
import frc.robot.commands.System.PlaceHigh;
import frc.robot.commands.System.PlaceMid;
import frc.robot.commands.System.ResetToBottom;
import frc.robot.commands.Auto.RedSide.RedCubePlaceAndMove;
import frc.robot.commands.Auto.RedSide.RedDirectBalance;
import frc.robot.commands.Auto.RedSide.RedLeftConePlaceAndGrab;
import frc.robot.commands.Auto.RedSide.RedPlaceAndBalance;
import frc.robot.commands.Auto.RedSide.RedRightCubePlaceAndGrab;
import frc.robot.commands.Auto.BlueSide.BlueDirectBalance;
import frc.robot.commands.Auto.BlueSide.BlueLeftCubePlaceAndGrab;
import frc.robot.commands.Auto.BlueSide.BlueRightConePlaceAndGrab;
import frc.robot.commands.Auto.BlueSide.BluePlaceAndBalance;
import frc.robot.commands.Clamp.SetClampLow;
import frc.robot.commands.Clamp.SetClampTop;
import frc.robot.commands.Clamp.SetClampTopHold;
import frc.robot.commands.Clamp.SpitClamp;
import frc.robot.commands.Clamp.SuckClamp;
import frc.robot.commands.Clamp.SetClampMid;
import frc.robot.commands.Clamp.SetClampCube;
import frc.robot.commands.Clamp.CloseClamp;
import frc.robot.commands.Clamp.ControllerClamp;
import frc.robot.commands.Clamp.OpenClamp;
import frc.robot.commands.Clamp.SetClampCone;
import frc.robot.commands.Intake.IntakeDown;
import frc.robot.commands.Intake.IntakeUp;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.RunIntakeReverse;
import frc.robot.commands.Elevator.ControllerElevator;
import frc.robot.commands.Elevator.ElevatorBottom;
import frc.robot.commands.Elevator.ElevatorConeMid;
import frc.robot.commands.Elevator.ElevatorFeederStation;
import frc.robot.commands.Elevator.TiltElevatorDown;
import frc.robot.commands.Elevator.ElevatorHigh;
import frc.robot.commands.Elevator.ElevatorSmallRaise;
import frc.robot.commands.Elevator.ElevatorTeleopLimit;
import frc.robot.commands.Elevator.ElevatorUpMid;
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

  //System 
  private final Command placeHigh = new PlaceHigh();
  private final Command resetToBottom = new ResetToBottom();
  private final Command blankCommand = new BlankCommand(robotDrive);
  private final Command placeMid = new PlaceMid();

  // Elevator
  private final Command tiltElevatorUp = new TiltElevatorUp(elevator);
  private final Command tiltElevatorDown = new TiltElevatorDown(elevator);
  private final Command elevatorHigh = new ElevatorHigh(elevator);
  private final Command elevatorMid = new ElevatorUpMid(elevator);
  private final Command elevatorTeleopLimit = new ElevatorTeleopLimit(elevator);
  private final Command elevatorBottom = new ElevatorBottom(elevator);
  private final Command elevatorSmallRaise = new ElevatorSmallRaise(elevator);
  private final Command elevatorFeederStationPosition = new FeederStationPosition();
  private final Command elevatorConeMidPosition = new ConeMidPosition();
  private final Command controllerElevator = new ControllerElevator(elevator, gunnerController);

  // Clamp
  private final Command setClampTop = new SetClampTop(clamp);
  private final Command setClampTopHold = new SetClampTopHold(clamp);
  private final Command setClampMid = new SetClampMid(clamp);
  private final Command setClampLow = new SetClampLow(clamp);
  private final Command setClampCube = new SetClampCube(clamp);
  private final Command setClampCone = new SetClampCone(clamp);
  private final Command controllerClamp = new ControllerClamp(clamp, gunnerController);
  private final Command openClamp = new OpenClamp();
  private final Command closeClamp = new CloseClamp();
  private final Command suckClamp = new SuckClamp();
  private final Command spitClamp = new SpitClamp();


  // Swerve
  private final ControllerDrive controllerDrive = new ControllerDrive(robotDrive, driverController);
  private final Command alignToTarget = new AlignToTarget(robotDrive, camera, driverController);
  private final Command alignToZero = new AlignToZero(robotDrive, driverController);
  private final Command moveToTarget = new MoveToTarget(robotDrive, camera);
  private final Command autoMoveToTarget = new AutoMoveToTarget(robotDrive, camera);
  private final Command autoBalance = new AutoBalance(robotDrive);
  private final Command formX = new FormX(robotDrive);

  //Auto
  private final Command bluePlaceAndBalance = new BluePlaceAndBalance();
  private final Command blueDirectBalance = new BlueDirectBalance();
  private final Command blueRightConePlaceAndGrab = new BlueRightConePlaceAndGrab();
  private final Command blueLeftCubePlaceAndGrab = new BlueLeftCubePlaceAndGrab();

  private final Command redPlaceAndBalance = new RedPlaceAndBalance();
  private final Command redDirectBalance = new RedDirectBalance();
  private final Command redLeftConePlaceAndGrab = new RedLeftConePlaceAndGrab();
  private final Command redRightCubePlaceAndGrab = new RedRightCubePlaceAndGrab();
  private final Command redCubePlaceAndMove = new RedCubePlaceAndMove();
  

  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    elevator.setDefaultCommand(controllerElevator);
    clamp.setDefaultCommand(controllerClamp);

    chooser.setDefaultOption("Red Direct Balance", redDirectBalance);
    chooser.addOption("Red Place and Balance", redPlaceAndBalance);
    chooser.addOption("Red Left Cone Place And Grab", redLeftConePlaceAndGrab);
    chooser.addOption("Red Right Cube Place And Grab", redRightCubePlaceAndGrab);
    chooser.addOption("Red Cube Place And Move", redCubePlaceAndMove);
    chooser.addOption("Blue Direct Balance", blueDirectBalance);
    chooser.addOption("Blue Place and Balance", bluePlaceAndBalance);
    chooser.addOption("Blue Right Cone Place And Grab", blueRightConePlaceAndGrab);
    chooser.addOption("Blue Left Cube Place And Grab", blueLeftCubePlaceAndGrab);

    // Put the chooser on the dashboard
    SmartDashboard.putData(chooser);
  }

  public void setTeleopDefaultCommands() {
    robotDrive.setDefaultCommand(controllerDrive);
  }

  public void initMethods() {
    elevator.resetElevatorEncoder();
    clamp.resetClampEncoder();
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
    driverController.x().whileTrue(formX);
    driverController.leftBumper().whileTrue(runIntake);
    driverController.rightBumper().whileTrue(runIntakeReverse);
    driverController.start().whileTrue(moveToTarget);
    driverController.back().whileTrue(autoMoveToTarget);
    driverController.povLeft().whileTrue(new InstantCommand(() -> camera.setPipeline(0)));
    driverController.povRight().whileTrue(new InstantCommand(() -> camera.setPipeline(1)));
    driverController.povUp().whileTrue(autoBalance);
    driverController.povDown().whileTrue(setClampTop);

    // Gunner Button Bindings
    gunnerController.y().whileTrue(tiltElevatorUp);
    gunnerController.a().whileTrue(tiltElevatorDown);
    gunnerController.b().whileTrue(openClamp);
    gunnerController.x().whileTrue(closeClamp);
    gunnerController.leftBumper().whileTrue(suckClamp);
    gunnerController.rightBumper().whileTrue(spitClamp);
    gunnerController.rightTrigger().whileTrue(placeHigh);  
    gunnerController.leftTrigger().whileTrue(resetToBottom);
    gunnerController.start().whileTrue(new InstantCommand(() -> clamp.resetClampEncoder()));
    gunnerController.back().whileTrue(new InstantCommand(() -> elevator.resetElevatorEncoder()));
    gunnerController.povUp().whileTrue(elevatorConeMidPosition);
    gunnerController.povDown().whileTrue(elevatorFeederStationPosition);
    gunnerController.povLeft().whileTrue(setClampCube);
    gunnerController.povRight().whileTrue(placeMid);
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
