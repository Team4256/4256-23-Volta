// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class SystemControl extends SubsystemBase {
  
  private double elevatorTargetHeight;
  private double clampTargetHeight;
  private Intake intake = Intake.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private Clamp clamp = Clamp.getInstance();

  public enum SystemStates {
    DEFAULT, INTAKING, PLACING_HIGH, PLACING_MID, STOP;
}

SystemStates currentState = SystemStates.STOP;
  /** Creates a new StateValidation. */
  public SystemControl() {
  
  }

  public void defaultConfig() {
    currentState = SystemStates.DEFAULT;
    elevatorTargetHeight = Constants.ELEVATOR_POSITION_BOTTOM;
    clampTargetHeight = Constants.CLAMP_LOW_POSITION;

}

public void intakingConfig() {
  currentState = SystemStates.INTAKING;
  elevatorTargetHeight = Constants.ELEVATOR_POSITION_BOTTOM;
  clampTargetHeight = Constants.CLAMP_LOW_POSITION;
}

public void placingHighConfig() {
  currentState = SystemStates.PLACING_HIGH;
  elevatorTargetHeight = Constants.ELEVATOR_POSITION_HIGH;
  clampTargetHeight = Constants.CLAMP_TOP_POSITION;
}

public void placingMidConfig() {
  currentState = SystemStates.PLACING_MID;
  elevatorTargetHeight = Constants.ELEVATOR_POSITION_MID;
  clampTargetHeight = Constants.CLAMP_TOP_POSITION;
}


public void runSystemControl() {

  switch (currentState) {
    case DEFAULT:
        defaultConfig();
        intake.intakeUp();
        intake.stop();
        elevator.setElevatorBottom();
        elevator.tiltElevatorUp();
        clamp.clampBottom();
        clamp.unclamp();
        break;
    case INTAKING:
        intakingConfig();
        intake.intakeDown();
        intake.stop();
        elevator.setElevatorBottom();
        elevator.tiltElevatorUp();
        clamp.clampBottom();
        clamp.unclamp();
        break;
    case PLACING_HIGH:
        placingHighConfig();
    case PLACING_MID:
        placingMidConfig();   
        break;
}


}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
