// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Xbox extends SubsystemBase {

  CommandXboxController controller;
  public Trigger y = controller.y();
  public Trigger a = controller.a();
  public Trigger x = controller.x();
  public Trigger b = controller.b();
  public Trigger start = controller.start();
  public Trigger back = controller.back();
  public Trigger rightBumper = controller.rightBumper();
  public Trigger leftBumper = controller.leftBumper();
  public double rightTrigger = controller.getRightTriggerAxis();
  public double leftTrigger = controller.getLeftTriggerAxis();
  public Trigger rightStickButton = controller.rightStick();
  public Trigger leftStickButton = controller.leftStick();
  public double leftStickX = controller.getLeftX();
  public double leftStickY = controller.getLeftY();
  public double rightStickX = controller.getRightX();
  public double rightStickY = controller.getRightY();
  public Trigger dPadUp = controller.povUp();
  public Trigger dPadLeft = controller.povLeft();
  public Trigger dPadRight = controller.povRight();
  public Trigger dPadDown = controller.povDown();

  /** Creates a new Xbox. */
  public Xbox(int port) {
    this.controller = new CommandXboxController(port);
    
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
