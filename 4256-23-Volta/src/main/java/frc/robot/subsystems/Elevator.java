// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private DigitalInput elevatorBottomLimitSwitch;
  private TalonSRX leftElevatorMotor;
  private TalonSRX rightElevatorMotor;
  private DoubleSolenoid elevatorSolenoid;
  public static Elevator instance = null;
  public double targetHeight = 0;
  private final TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(0.0, 0.0);
  private final ProfiledPIDController pidController = new ProfiledPIDController(0.0, 0.0, 0.0, pidConstraints, 0.02);

  /** Creates a new Elevator. */
  public Elevator() {

    this.leftElevatorMotor = new TalonSRX(Constants.ELEVATOR_LEFT_MOTOR_ID);
    this.rightElevatorMotor = new TalonSRX(Constants.ELEVATOR_RIGHT_MOTOR_ID);
    this.elevatorSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.ELEVATOR_SOLENOID_FORWARD_CHANNEL, Constants.ELEVATOR_SOLENOID_REVERSE_CHANNEL);
    this.elevatorBottomLimitSwitch = new DigitalInput(Constants.ELEVATOR_BOTTOM_LIMIT_SWITCH_ID);

    configElevatorMotor();
  }

  public static synchronized Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();

    }
    return instance;
  }

  public double getElevatorEncoderPosition() {
    return leftElevatorMotor.getSelectedSensorPosition();
  }

  public void resetElevatorEncoder() {
    leftElevatorMotor.setSelectedSensorPosition(0);
  }

  public void setElevatorHigh() {

    if (getElevatorEncoderPosition() <= Constants.ELEVATOR_UPPER_LIMIT) {
      
      double speed = pidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_POSITION_HIGH);
      leftElevatorMotor.set(ControlMode.PercentOutput, speed);
      leftElevatorMotor.set(ControlMode.PercentOutput, -speed);
    } else {
      stopElevator();
    }

  }

  public void setElevatorMid() {
    if (getElevatorEncoderPosition() <= Constants.ELEVATOR_UPPER_LIMIT) {
      double speed = pidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_POSITION_MID);
      leftElevatorMotor.set(ControlMode.PercentOutput, speed);
      leftElevatorMotor.set(ControlMode.PercentOutput, -speed);
    } else {
      stopElevator();
    }
  }

  public void setElevatorLow() {
    if (getElevatorEncoderPosition() <= Constants.ELEVATOR_UPPER_LIMIT) {
      double speed = pidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_POSITION_LOW);
      leftElevatorMotor.set(ControlMode.PercentOutput, speed);
      leftElevatorMotor.set(ControlMode.PercentOutput, -speed);
    } else {
      stopElevator();
    }
  }

  public void setElevatorBottom() {
    if (getElevatorEncoderPosition() <= Constants.ELEVATOR_UPPER_LIMIT) {
      double speed = pidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_POSITION_BOTTOM);
      leftElevatorMotor.set(ControlMode.PercentOutput, speed);
      leftElevatorMotor.set(ControlMode.PercentOutput, -speed);
    } else {
      stopElevator();
    }
  }

  public void incrementElevator() {
    targetHeight = getElevatorEncoderPosition() + 5;
    if ((getElevatorEncoderPosition() <= Constants.ELEVATOR_UPPER_LIMIT)
        && (getElevatorEncoderPosition() >= Constants.ELEVATOR_BOTTOM_LIMIT)) {
          double speed = pidController.calculate(getElevatorEncoderPosition(), targetHeight);
          leftElevatorMotor.set(ControlMode.PercentOutput, speed);
          leftElevatorMotor.set(ControlMode.PercentOutput, -speed);
    } else {
      stopElevator();
    }
  }

  public void decrementElevator() {
    targetHeight = getElevatorEncoderPosition() - 5;
    if ((getElevatorEncoderPosition() <= Constants.ELEVATOR_UPPER_LIMIT)
        && (getElevatorEncoderPosition() <= Constants.ELEVATOR_BOTTOM_LIMIT)) {
          double speed = pidController.calculate(getElevatorEncoderPosition(), targetHeight);
          leftElevatorMotor.set(ControlMode.PercentOutput, speed);
          leftElevatorMotor.set(ControlMode.PercentOutput, -speed);
    } else {
      stopElevator();
    }
  }

  public void setElevatorMotor(double speed) {

    if (Math.abs(speed) > .4) {
      speed = .4;

    }

    // if ((speed >= .1) && !elevatorBottomLimitSwitch.get()) {
    //   if ((getElevatorEncoderPosition() <= Constants.ELEVATOR_UPPER_LIMIT)
    //       && (getElevatorEncoderPosition() >= Constants.ELEVATOR_BOTTOM_LIMIT)) {
    //     leftElevatorMotor.set(ControlMode.PercentOutput, speed);
    //     rightElevatorMotor.set(ControlMode.PercentOutput, -speed);
    //   } else {
    //     stopElevator();
    //   }
    // } else {
    //   stopElevator();
    // }

      // if ((speed > 0) && !elevatorBottomLimitSwitch.get()) {
      //   leftElevatorMotor.set(ControlMode.PercentOutput, speed);
      //   rightElevatorMotor.set(ControlMode.PercentOutput, -speed);
      // } else if ((speed < 0) && !elevatorBottomLimitSwitch.get()) {
      //   stopElevator();
      // } else {
      //   leftElevatorMotor.set(ControlMode.PercentOutput, speed);
      //   rightElevatorMotor.set(ControlMode.PercentOutput, -speed);
      // }

        leftElevatorMotor.set(ControlMode.PercentOutput, speed);
        rightElevatorMotor.set(ControlMode.PercentOutput, -speed);
      
      SmartDashboard.putNumber("elevator speed", speed);
    
  }

  public void stopElevator() {
    leftElevatorMotor.set(ControlMode.PercentOutput, 0);
    rightElevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public void tiltElevatorDown() {
    elevatorSolenoid.set(Value.kForward);
  }

  public void tiltElevatorUp() {
    elevatorSolenoid.set(Value.kReverse);
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
    SmartDashboard.putNumber("Elevator Position", getElevatorEncoderPosition());
    SmartDashboard.putBoolean("Elevator Limit Switch", elevatorBottomLimitSwitch.get());

  }
}