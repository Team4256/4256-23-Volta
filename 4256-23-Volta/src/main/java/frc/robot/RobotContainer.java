// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Swerve.AlignToTarget;
import frc.robot.commands.Swerve.AlignToZero;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.ControllerDrive;
import frc.robot.commands.Swerve.FormX;
import frc.robot.commands.Swerve.MoveToTarget;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_ID);
  XboxController operatorController = new XboxController(Constants.GUNNER_CONTROLLER_ID);
  private final SwerveSubsystem robotDrive = new SwerveSubsystem();
  private final ControllerDrive swerveDrive = new ControllerDrive(robotDrive, driverController);
  private final Limelight camera = new Limelight();
  private final Command alignToTarget = new AlignToTarget(robotDrive, camera, driverController);
  private final Command alignToZero = new AlignToZero(robotDrive, camera, driverController);
  private final Command moveToTarget = new MoveToTarget(robotDrive, camera, driverController);
  private final Command autoBalance = new AutoBalance(robotDrive, camera, driverController);
  private final Command formX = new FormX(robotDrive);
  private final Gyro gyro = Gyro.getInstance();

  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    robotDrive.setDefaultCommand(swerveDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   
    new JoystickButton(driverController, Button.kA.value).whileTrue(autoBalance);
    new JoystickButton(driverController, Button.kY.value).whileTrue(moveToTarget);
    new JoystickButton(driverController, Button.kB.value).onTrue(new InstantCommand(() -> gyro.reset()));
    new JoystickButton(driverController, Button.kX.value).whileTrue(formX);
    new JoystickButton(driverController, Button.kLeftBumper.value).whileTrue(alignToZero); 
    
  }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(swerveDrive);
  }
}
