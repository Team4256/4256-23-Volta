// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator2. */
  private final VictorSPX baseMotor1;
  private final VictorSPX baseMotor2;
  private final DoubleSolenoid tiltSolenoid;

  
  public Elevator() {
    baseMotor1 = new VictorSPX(Constants.MOTOR_ID_PLACEHOLDER);
    baseMotor2 = new VictorSPX(Constants.MOTOR_ID_PLACEHOLDER);
    tiltSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.TILT_FORWARD_PNEUMATIC_ID,
        Constants.TILT_REVERSE_PNEUMATIC_ID);
  }

  public void tiltElevator() {
  
  }
  public void activateClamp() {

  }
  public void extendElevatorTop() {
    baseMotor1.set(ControlMode.PercentOutput, Constants.EXTENSION_MOTOR_SPEED);
  }

  public void retractElevatorBottom() {
    baseMotor1.set(ControlMode.PercentOutput, Constants.RETRACTION_MOTOR_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
