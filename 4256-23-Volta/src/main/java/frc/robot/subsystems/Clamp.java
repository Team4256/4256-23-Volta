// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Clamp extends SubsystemBase {

  private DoubleSolenoid solenoid;
  private DutyCycleEncoder clampEncoder;
  private VictorSPX clampMotor;
  private final TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(0.0, 0.0);
  private final ProfiledPIDController pidController = new ProfiledPIDController(0.0, 0.0, 0.0, pidConstraints, 0.02);
  public static Clamp instance = null;
  /** Creates a new Clamp. */
  public Clamp() {
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLAMP_SOLENOID_FORWARD_CHANNEL, Constants.CLAMP_SOLENOID_REVERSE_CHANNEL);
    this.clampEncoder = new DutyCycleEncoder(6);
    this.clampMotor = new VictorSPX(Constants.CLAMP_MOTOR_ID);
    configClampMotor();
  }

  public static synchronized Clamp getInstance() {
		if (instance == null) {
			instance = new Clamp();
			
		} 
		return instance;
	}

  public void clamp() {
    solenoid.set(Value.kForward);
  }

  public void unclamp() {
    solenoid.set(Value.kReverse);
  }

  public void clampBottom() {

    if (clampEncoder.get() >= Constants.CLAMP_LOWER_LIMIT) {
      double speed = pidController.calculate(clampEncoder.get(), Constants.CLAMP_BOTTOM_POSITION);
      clampMotor.set(ControlMode.PercentOutput, speed);
    } else {
      stop();
    }

  }

  public void clampMid() {

    if ((clampEncoder.get() >= Constants.CLAMP_LOWER_LIMIT) && (clampEncoder.get() <= Constants.CLAMP_UPPER_LIMIT)) {
      double speed = pidController.calculate(clampEncoder.get(), Constants.CLAMP_MID_POSITION);
      clampMotor.set(ControlMode.PercentOutput, speed);
    } else {
      stop();
    }

  }

  public void clampTop() {
    if ((clampEncoder.get() >= Constants.CLAMP_LOWER_LIMIT) && (clampEncoder.get() <= Constants.CLAMP_UPPER_LIMIT)) {
      double speed = pidController.calculate(clampEncoder.get(), Constants.CLAMP_TOP_POSITION);
      clampMotor.set(ControlMode.PercentOutput, speed);
    } else {
      stop();
    }
  }

  public void setClampSpeed(double speed) {

  //   if (speed >= .1) {
  //   if ((clampEncoder.get() >= Constants.CLAMP_LOWER_LIMIT) && (clampEncoder.get() <= Constants.CLAMP_UPPER_LIMIT)) {
  //     clampMotor.set(ControlMode.PercentOutput, speed);
  //   } else {
  //     stop();
  //   }
  // } else {
  //   stop();
  // }
  clampMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    clampMotor.set(ControlMode.PercentOutput, 0);
  }

  private void configClampMotor() {
    clampMotor.configFactoryDefault();
        clampMotor.config_kP(0, Constants.CLAMP_MOTOR_KP);
        clampMotor.config_kI(0, Constants.CLAMP_MOTOR_KI);
        clampMotor.config_kD(0, Constants.CLAMP_MOTOR_KD);
        clampMotor.config_kF(0, Constants.CLAMP_MOTOR_KF);
        clampMotor.setInverted(false);
        clampMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}