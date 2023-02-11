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
import frc.robot.commands.Clamp.ClampBottom;
import frc.robot.commands.Clamp.ClampHigh;
import frc.robot.commands.Clamp.ClampMid;
import frc.robot.commands.Clamp.ControllerClamp;
import frc.robot.commands.Elevator.IncrementElevator;
import frc.robot.commands.Elevator.ElevatorBottom;
import frc.robot.commands.Elevator.ElevatorHigh;
import frc.robot.commands.Elevator.ElevatorLow;
import frc.robot.commands.Elevator.ElevatorMid;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_ID);
  XboxController gunnerController = new XboxController(Constants.GUNNER_CONTROLLER_ID);
  private final SwerveSubsystem robotDrive = new SwerveSubsystem();
  private final Elevator elevator = Elevator.getInstance();
  private final Clamp clamp = Clamp.getInstance();
  private final Limelight camera = new Limelight();
  private final Gyro gyro = Gyro.getInstance();

  // Elevator
  private final Command elevatorHigh = new ElevatorHigh(elevator);
  private final Command elevatorMid = new ElevatorMid(elevator);
  private final Command elevatorLow = new ElevatorLow(elevator);
  private final Command elevatorBottom = new ElevatorBottom(elevator);
  private final Command incrementElevator = new IncrementElevator(elevator);

  // Clamp
  private final Command clampHigh = new ClampHigh(clamp);
  private final Command clampMid = new ClampMid(clamp);
  private final Command clampBottom = new ClampBottom(clamp);
  private final Command controllerClamp = new ControllerClamp(clamp, gunnerController);

  // Swerve
  private final ControllerDrive controllerDrive = new ControllerDrive(robotDrive, driverController);
  private final Command alignToTarget = new AlignToTarget(robotDrive, camera, driverController);
  private final Command alignToZero = new AlignToZero(robotDrive, camera, driverController);
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

    chooser.setDefaultOption("Direct Balance", directBalance);
    chooser.addOption("Two Cone Auto Top", twoConeAutoTop);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Competition").add(chooser);
  }

  public void setTeleopSwerveDefaultCommand() {
    robotDrive.setDefaultCommand(controllerDrive);
  }

  public void setAutoSwerveDefaultCommand() {
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

    //Driver Button Bindings
/**
 * A: Auto Balance
 * B: Reset Gyro
 * Y: Move To Target
 * X: Form X
 * Left Bumper: Align To Zero
 * Right Bumper: Reset Odometer
 */
    new JoystickButton(driverController, Button.kA.value).whileTrue(autoBalance);
    new JoystickButton(driverController, Button.kB.value).onTrue(new InstantCommand(() -> gyro.reset()));
    new JoystickButton(driverController, Button.kY.value).whileTrue(moveToTarget);
    new JoystickButton(driverController, Button.kX.value).whileTrue(formX);
    new JoystickButton(driverController, Button.kLeftBumper.value).whileTrue(alignToZero);
    new JoystickButton(driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> robotDrive.resetOdometerToZero()));

        //Gunner Button Bindings
/*
 * Y: Elevator High
 * X: Elevator Mid
 * B: Elevator Low
 * A: Elevator Bottom
 * Start: Increment Elevator
 * Back: Controller Clamp
 * Left Bumper: Clamp High
 * Right Bumper: Clamp Mid
 * Right Stick: Clamp Bottom
 * 
 */
    new JoystickButton(gunnerController, Button.kY.value).whileTrue(elevatorHigh);
    new JoystickButton(gunnerController, Button.kX.value).whileTrue(elevatorMid);
    new JoystickButton(gunnerController, Button.kB.value).whileTrue(elevatorLow);
    new JoystickButton(gunnerController, Button.kA.value).whileTrue(elevatorBottom);
    new JoystickButton(gunnerController, Button.kStart.value).whileTrue(incrementElevator);

    new JoystickButton(gunnerController, Button.kLeftBumper.value).whileTrue(clampHigh);
    new JoystickButton(gunnerController, Button.kRightBumper.value).whileTrue(clampMid);
    new JoystickButton(gunnerController, Button.kRightStick.value).whileTrue(clampBottom);
    new JoystickButton(gunnerController, Button.kBack.value).whileTrue(controllerClamp);

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
