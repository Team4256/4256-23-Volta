// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  private DutyCycleEncoder elevatorEncoder;
  private TalonFX elevatorMotor;
  
  /** Creates a new Elevator. */
  public Elevator() {
    this.elevatorEncoder = new DutyCycleEncoder(0);
    this.elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR_ID);
    configElevatorMotor();
  }

  public double getElevatorEncoderPosition() {
    return elevatorEncoder.getAbsolutePosition();
  }

  public void resetElevatorEncoder() {
    elevatorEncoder.reset();
  }

  public void setElevatorHigh() {
    if (elevatorEncoder.get() < Constants.ELEVATOR_UPPER_LIMIT) {
      elevatorMotor.set(ControlMode.Position, Constants.ELEVATOR_POSITION_HIGH);
    } else {
      stopElevator();
    }
  }

  public void setElevatorMid() {

  }

  public void setElevatorLow() {

  }

  public void setElevatorBottom() {

  }

  public void stopElevator() {

  }

  private void configElevatorMotor() {
    elevatorMotor.configFactoryDefault();
        elevatorMotor.config_kP(0, Constants.ELEVATOR_MOTOR_KP);
        elevatorMotor.config_kI(0, Constants.ELEVATOR_MOTOR_KI);
        elevatorMotor.config_kD(0, Constants.ELEVATOR_MOTOR_KD);
        elevatorMotor.config_kF(0, Constants.ELEVATOR_MOTOR_KF);
        elevatorMotor.setInverted(false);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
