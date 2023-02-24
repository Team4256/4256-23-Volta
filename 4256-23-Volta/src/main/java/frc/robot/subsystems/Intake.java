// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private DoubleSolenoid solenoid;
  private VictorSPX intakeMotor;
  public static Intake instance = null;
  /** Creates a new Clamp. */
  public Intake() {
    this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_FORWARD_CHANNEL , Constants.INTAKE_SOLENOID_REVERSE_CHANNEL);
    this.intakeMotor = new VictorSPX(Constants.INTAKE_MOTOR_ID);
    configIntakeMotor();
  }

  public static synchronized Intake getInstance() {
		if (instance == null) {
			instance = new Intake();
			
		} 
		return instance;
	}

  public void suck() {
    intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_MOTOR_SPEED);
  }

  public void spit() {
    intakeMotor.set(ControlMode.PercentOutput, -Constants.INTAKE_MOTOR_SPEED);
  }

  public void intakeUp() {
    solenoid.set(Value.kForward);
  }

  public void intakeDown() {
    solenoid.set(Value.kReverse);
  }
  
  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  private void configIntakeMotor() {
    intakeMotor.configFactoryDefault();
        intakeMotor.config_kP(0, Constants.CLAMP_MOTOR_KP);
        intakeMotor.config_kI(0, Constants.CLAMP_MOTOR_KI);
        intakeMotor.config_kD(0, Constants.CLAMP_MOTOR_KD);
        intakeMotor.config_kF(0, Constants.CLAMP_MOTOR_KF);
        intakeMotor.setInverted(false);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
