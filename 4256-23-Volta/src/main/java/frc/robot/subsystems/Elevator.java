package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Elevator {

    private static DoubleSolenoid tiltSolenoid;
    private final VictorSPX baseMotor1;
    private final VictorSPX baseMotor2;
    
    
    public Elevator(){
        baseMotor1 = new VictorSPX(Constants.MOTOR_ID_PLACEHOLDER);
        baseMotor2 = new VictorSPX(Constants.MOTOR_ID_PLACEHOLDER);
        tiltSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PNEUMATIC_ID_PLACEHOLDER, Constants.PNEUMATIC_ID_PLACEHOLDER);
    }
    
    public void tiltElevator() {
        tiltSolenoid.set(Value.kForward); 
    }

    public void resetTiltElevator() {
        tiltSolenoid.set(Value.kReverse);
    }

    public void extendElevatorTop() {
        baseMotor1.set(ControlMode.PercentOutput, Constants.EXTENSION_MOTOR_SPEED);
    }

    public void retractElevatorBottom() {
        baseMotor1.set(ControlMode.PercentOutput, Constants.RETRACTION_MOTOR_SPEED);
    }

    public void setElevator(int encoderCounts) {

    }



    
}
