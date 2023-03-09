// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//Note, VictorSPX encoders have 4096 ticks per rotation

public class Clamp extends SubsystemBase {

  private DoubleSolenoid solenoid;
  private VictorSPX clampMotor;
  private VictorSPX intakeMotor;
  public static Clamp instance = null;
  private CANCoder clampCoder;
  private PIDController clampPidController;
  private DigitalInput clampLimitSwitch;

  public Clamp() {
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLAMP_SOLENOID_FORWARD_CHANNEL,
        Constants.CLAMP_SOLENOID_REVERSE_CHANNEL);
    this.clampMotor = new VictorSPX(Constants.CLAMP_MOTOR_ID);
    this.intakeMotor = new VictorSPX(Constants.CLAMP_INTAKE_MOTOR_ID);
    this.clampCoder = new CANCoder(Constants.CLAMP_ENCODER_ID);
    this.clampPidController = new PIDController(Constants.CLAMP_MOTOR_KP, Constants.CLAMP_MOTOR_KI,
        Constants.CLAMP_MOTOR_KD);
    this.clampLimitSwitch = new DigitalInput(Constants.CLAMP_LIMIT_SWITCH_ID);
    configClampMotor();
  }

  public static synchronized Clamp getInstance() {
    if (instance == null) {
      instance = new Clamp();

    }
    return instance;
  }
  public boolean getClampLimitSwitch() {

    if (clampLimitSwitch.get()) {
      return false;
    } else {
      return true;
    }

  }
  public double getCANCoderAngle() {
    return clampCoder.getPosition();
  }

  public void resetClampEncoder() {
    clampCoder.setPosition(0);
  }

  public void clamp() {
    solenoid.set(Value.kReverse);
  }

  public void unclamp() {
    solenoid.set(Value.kForward);
  }

  public void suck() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, -Constants.CLAMP_INTAKE_MOTOR_SPEED);
  }

  public void spit() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, Constants.CLAMP_INTAKE_MOTOR_SPEED);
  }

  public void stopIntake() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }
  public void setClampTop() {

    double speed = clampPidController.calculate(getCANCoderAngle(), Constants.CLAMP_TOP_POSITION_1);
    if (Math.abs(speed) > .8) {
      speed = .8 * Math.signum(speed);
    }

    clampMotor.set(ControlMode.PercentOutput, -speed);
  }

  public void setClampMid() {

    double speed = clampPidController.calculate(getCANCoderAngle(), Constants.CLAMP_MID_POSITION);
    if (Math.abs(speed) > .8) {
      speed = .8 * Math.signum(speed);
    }

    clampMotor.set(ControlMode.PercentOutput, -speed);
  }

  public void setClampLow() {
    double speed = clampPidController.calculate(getCANCoderAngle(), Constants.CLAMP_LOW_POSITION);
    if (Math.abs(speed) > .8) {
      speed = .8 * Math.signum(speed);
    }

    clampMotor.set(ControlMode.PercentOutput, -speed);
  }

  public void setClampGrab() {
    double speed = clampPidController.calculate(getCANCoderAngle(), Constants.CLAMP_GRAB_POSITION);
    if (Math.abs(speed) > .8) {
      speed = .8 * Math.signum(speed);
    }

    clampMotor.set(ControlMode.PercentOutput, -speed);
  }

  public void setClampSpeed(double speed) {
       clampMotor.set(VictorSPXControlMode.PercentOutput, -speed);
  }

  public void stopInnerClamp() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void stop() {
    clampMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  private void configClampMotor() {
    clampMotor.configFactoryDefault();
    clampMotor.config_kP(0, Constants.CLAMP_MOTOR_KP);
    clampMotor.config_kI(0, Constants.CLAMP_MOTOR_KI);
    clampMotor.config_kD(0, Constants.CLAMP_MOTOR_KD);
    clampMotor.setInverted(false);
    clampMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClampCoder Angle", getCANCoderAngle());
    SmartDashboard.putNumber("Clamp Motor Voltage", clampMotor.getMotorOutputVoltage());
    SmartDashboard.putBoolean("Clamp Limit Switch", getClampLimitSwitch());
  }
}