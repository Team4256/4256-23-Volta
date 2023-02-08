package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Elevator {

    private static DoubleSolenoid tiltSolenoid;
    
    
    public Elevator(){
        
        tiltSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PNEUMATIC_ID_PLACEHOLDER, Constants.PNEUMATIC_ID_PLACEHOLDER);
    }
    
    public void tiltElevator() {
        tiltSolenoid.set(Value.kForward); 
    }

    public void resetTiltElevator() {
        tiltSolenoid.set(Value.kReverse);
    }

    public void extendElevatorTop() {
        
    }

    public void retractElevatorBottom() {
        
    }



    
}
