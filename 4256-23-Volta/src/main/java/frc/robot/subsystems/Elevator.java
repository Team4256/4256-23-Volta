// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private DigitalInput elevatorTopLimitSwitch;
  private DigitalInput elevatorBottomLimitSwitch;
  private TalonSRX leftElevatorMotor;
  private TalonSRX rightElevatorMotor;
  private DoubleSolenoid elevatorSolenoid;
  private PIDController elevatorPidController;
  private double targetHeight;
  public static Elevator instance = null;

  public Elevator() {
    this.leftElevatorMotor = new TalonSRX(Constants.ELEVATOR_LEFT_MOTOR_ID);
    this.rightElevatorMotor = new TalonSRX(Constants.ELEVATOR_RIGHT_MOTOR_ID);
    this.elevatorSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        Constants.ELEVATOR_SOLENOID_FORWARD_CHANNEL, Constants.ELEVATOR_SOLENOID_REVERSE_CHANNEL);
    this.elevatorTopLimitSwitch = new DigitalInput(Constants.ELEVATOR_TOP_LIMIT_SWITCH_ID);
    this.elevatorBottomLimitSwitch = new DigitalInput(Constants.ELEVATOR_BOTTOM_LIMIT_SWITCH_ID);
    this.elevatorPidController = new PIDController(Constants.ELEVATOR_MOTOR_KP, Constants.ELEVATOR_MOTOR_KI, Constants.ELEVATOR_MOTOR_KD);
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

  public boolean getElevatorBottomLimitSwitch() {

    if (elevatorBottomLimitSwitch.get()) {
      return false;
    } else {
      return true;
    }

  }

  public boolean getElevatorTopLimitSwitch() {

    if (elevatorTopLimitSwitch.get()) {
      return false;
    } else {
      return true;
    }

  }

  public void setElevatorHigh() {

    double speed = elevatorPidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_TOP_POSITION);
    if (Math.abs(speed) > .6) {
      speed = .6 * Math.signum(speed);
    }
    
    SmartDashboard.putNumber("Elevator Speed", speed);
    leftElevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setElevatorMid() {

    double speed = elevatorPidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_MID_POSITION);
    if (Math.abs(speed) > .6) {
      speed = .6 * Math.signum(speed);
    }
    
    leftElevatorMotor.set(ControlMode.PercentOutput, speed);
    
  }

  public void setElevatorTeleopLimit() {
    double speed = elevatorPidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_TELEOP_LIMIT_POSITION);
    if (Math.abs(speed) > .6) {
      speed = .6 * Math.signum(speed);
    }
    
    leftElevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setElevatorBottom() {
    double speed = elevatorPidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_BASE_POSITION);
    if (Math.abs(speed) > .6) {
      speed = .6 * Math.signum(speed);
    }
    
    leftElevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setElevatorSmallRaise() {
    double speed = elevatorPidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_SMALL_RAISE_POSITION);
    if (Math.abs(speed) > .6) {
      speed = .6 * Math.signum(speed);
    }
    
    leftElevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setElevatorFeederStation() {
    double speed = elevatorPidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_FEEDER_STATION_POSITION);
    if (Math.abs(speed) > .6) {
      speed = .6 * Math.signum(speed);
    }
    
    leftElevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setElevatorConeMid() {
    double speed = elevatorPidController.calculate(getElevatorEncoderPosition(), Constants.ELEVATOR_CONE_MID_POSITION);
    if (Math.abs(speed) > .6) {
      speed = .6 * Math.signum(speed);
    }
    
    leftElevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  // public void setElevatorMotor(double speed) {
  //  leftElevatorMotor.set(TalonSRXControlMode.PercentOutput, speed);
  // }

  public void setElevatorMotor(double speed) {

    if (getElevatorTopLimitSwitch() && speed < 0) {
      stopElevator();
    } else if (getElevatorBottomLimitSwitch() && speed > 0) {
      stopElevator();
    } else {
      leftElevatorMotor.set(ControlMode.PercentOutput, speed);
    }

   }

  public void stopElevator() {
    leftElevatorMotor.set(ControlMode.PercentOutput, 0);
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
    rightElevatorMotor.follow(leftElevatorMotor);
    rightElevatorMotor.setInverted(true);
    rightElevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    SmartDashboard.putNumber("Elevator Encoder Counts", getElevatorEncoderPosition());
    SmartDashboard.putBoolean("Elevator Top Limit Switch", getElevatorTopLimitSwitch());
    SmartDashboard.putBoolean("Elevator Bottom Limit Switch", getElevatorBottomLimitSwitch());
  }
}
