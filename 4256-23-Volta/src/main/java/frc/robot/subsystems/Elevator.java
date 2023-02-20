// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  private DutyCycleEncoder elevatorEncoder;
  private TalonSRX leftElevatorMotor;
  private TalonSRX rightElevatorMotor;
  private DoubleSolenoid elevatorSolenoid;
  public static Elevator instance = null;
  public double targetAngle = 0;
  private final TrapezoidProfile.Constraints pidConstraints =
    new TrapezoidProfile.Constraints(1.75, 0.75);
  private final ProfiledPIDController pidController =
    new ProfiledPIDController(1.3, 0.0, 0.7, pidConstraints, 0.02);

  /** Creates a new Elevator. */
  public Elevator() {
    this.elevatorEncoder = new DutyCycleEncoder(5);
    this.leftElevatorMotor = new TalonSRX(Constants.ELEVATOR_LEFT_MOTOR_ID);
    this.rightElevatorMotor = new TalonSRX(Constants.ELEVATOR_RIGHT_MOTOR_ID);
    this.elevatorSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ELEVATOR_SOLENOID_FORWARD_CHANNEL, Constants.ELEVATOR_SOLENOID_REVERSE_CHANNEL);
    
    configElevatorMotor();
  }

  public static synchronized Elevator getInstance() {
		if (instance == null) {
			instance = new Elevator();
			
		} 
		return instance;
	}

  public double getElevatorEncoderPosition() {
    return elevatorEncoder.getAbsolutePosition();
  }

  public void resetElevatorEncoder() {
    elevatorEncoder.reset();
  }

  public void setElevatorHigh() {

    if (elevatorEncoder.get() <= Constants.ELEVATOR_UPPER_LIMIT) {
      pidController.calculate(elevatorEncoder.get(), Constants.ELEVATOR_POSITION_HIGH);
    } else {
      stopElevator();
    }

  }

  public void setElevatorMid() {
    if (elevatorEncoder.get() <= Constants.ELEVATOR_UPPER_LIMIT) {
      pidController.calculate(elevatorEncoder.get(), Constants.ELEVATOR_POSITION_MID);
    } else {
      stopElevator();
    }
  }

  public void setElevatorLow() {
    if (elevatorEncoder.get() <= Constants.ELEVATOR_UPPER_LIMIT) {
      pidController.calculate(elevatorEncoder.get(), Constants.ELEVATOR_POSITION_LOW);
    } else {
      stopElevator();
    }
  }

  public void setElevatorBottom() {
    if (elevatorEncoder.get() <= Constants.ELEVATOR_UPPER_LIMIT) {
      leftElevatorMotor.set(ControlMode.Position, Constants.ELEVATOR_POSITION_BOTTOM);
    } else {
      stopElevator();
    }
  }

  public void incrementElevator() {
    targetAngle = elevatorEncoder.get() + 5;
    if ((elevatorEncoder.get() <= Constants.ELEVATOR_UPPER_LIMIT) && (elevatorEncoder.get() <= Constants.ELEVATOR_BOTTOM_LIMIT)) {
      leftElevatorMotor.set(ControlMode.PercentOutput, targetAngle);
    } else {
      stopElevator();
    }
  }

  public void decrementElevator() {
    targetAngle = elevatorEncoder.get() - 5;
    if ((elevatorEncoder.get() <= Constants.ELEVATOR_UPPER_LIMIT) && (elevatorEncoder.get() <= Constants.ELEVATOR_BOTTOM_LIMIT)) {
      leftElevatorMotor.set(ControlMode.PercentOutput, targetAngle);
    } else {
      stopElevator();
    }
  }

  public void setElevatorMotor(double speed) {

    if ((elevatorEncoder.get() <= Constants.ELEVATOR_UPPER_LIMIT) && (elevatorEncoder.get() <= Constants.ELEVATOR_BOTTOM_LIMIT)) {
      leftElevatorMotor.set(ControlMode.PercentOutput, speed);
    rightElevatorMotor.set(ControlMode.PercentOutput, -speed);
    } else {
      stopElevator();
    }
    
  }

  public void stopElevator() {
    leftElevatorMotor.set(ControlMode.PercentOutput, 0);
    rightElevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public void tiltElevatorDown() {
    elevatorSolenoid.set(Value.kForward);
  }
  
  public void tiltElevatorUp() {
    elevatorSolenoid.set(Value.kForward);
  }

  private void configElevatorMotor() {
    leftElevatorMotor.configFactoryDefault();
        leftElevatorMotor.config_kP(0, Constants.ELEVATOR_MOTOR_KP);
        leftElevatorMotor.config_kI(0, Constants.ELEVATOR_MOTOR_KI);
        leftElevatorMotor.config_kD(0, Constants.ELEVATOR_MOTOR_KD);
        leftElevatorMotor.config_kF(0, Constants.ELEVATOR_MOTOR_KF);
        leftElevatorMotor.setInverted(false);
        leftElevatorMotor.setNeutralMode(NeutralMode.Brake);

        rightElevatorMotor.configFactoryDefault();
        rightElevatorMotor.config_kP(0, Constants.ELEVATOR_MOTOR_KP);
        rightElevatorMotor.config_kI(0, Constants.ELEVATOR_MOTOR_KI);
        rightElevatorMotor.config_kD(0, Constants.ELEVATOR_MOTOR_KD);
        rightElevatorMotor.config_kF(0, Constants.ELEVATOR_MOTOR_KF);
        rightElevatorMotor.setInverted(false);
        rightElevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
