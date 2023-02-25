// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//Note, VictorSPX encoders have 4096 ticks per rotation

public class Clamp extends SubsystemBase {

  private DoubleSolenoid solenoid;
  private VictorSPX clampMotor;
  public static Clamp instance = null;

  public Clamp() {
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLAMP_SOLENOID_FORWARD_CHANNEL,
        Constants.CLAMP_SOLENOID_REVERSE_CHANNEL);
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

    
      clampMotor.set(VictorSPXControlMode.Position, Constants.CLAMP_BOTTOM_POSITION);
  

  }

  public void clampMid() {


      clampMotor.set(VictorSPXControlMode.Position, Constants.CLAMP_MID_POSITION);
  

  }

  public void clampTop() {
  
      clampMotor.set(VictorSPXControlMode.Position, Constants.CLAMP_TOP_POSITION);
    
  }

  public void setClampSpeed(double speed) {
    clampMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void stop() {
    clampMotor.set(VictorSPXControlMode.PercentOutput, 0);
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