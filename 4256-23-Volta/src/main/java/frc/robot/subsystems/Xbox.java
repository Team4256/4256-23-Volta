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
  public Trigger y ;
  public Trigger a;
  public Trigger x ;
  public Trigger b ;
  public Trigger start ;
  public Trigger back;
  public Trigger rightBumper ;
  public Trigger leftBumper ;
  public double rightTrigger ;
  public double leftTrigger;
  public Trigger rightStickButton;
  public Trigger leftStickButton ;
  public double leftStickX;
  public double leftStickY;
  public double rightStickX ;
  public double rightStickY ;
  public Trigger dPadUp ;
  public Trigger dPadLeft ;
  public Trigger dPadRight ;
  public Trigger dPadDown ;

  /** Creates a new Xbox. */
  public Xbox(int port) {
    controller = new CommandXboxController(port);
    
 y = controller.y();
a = controller.a();
x = controller.x();
b = controller.b();
  start = controller.start();
 back = controller.back();
rightBumper = controller.rightBumper();
leftBumper = controller.leftBumper();
 rightTrigger = controller.getRightTriggerAxis();
leftTrigger = controller.getLeftTriggerAxis();
rightStickButton = controller.rightStick();
leftStickButton = controller.leftStick();
 leftStickX = controller.getLeftX();
 leftStickY = controller.getLeftY();
 rightStickX = controller.getRightX();
 rightStickY = controller.getRightY();
 dPadUp = controller.povUp();
 dPadLeft = controller.povLeft();
 dPadRight = controller.povRight();
 dPadDown = controller.povDown();

  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
