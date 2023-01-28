package frc.robot.subsystems;

import java.time.chrono.ThaiBuddhistChronology;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Conversions;

public final class SwerveModule {
	public static final double ROTATOR_GEAR_RATIO = 1.0;
	public static final double TRACTION_GEAR_RATIO = 52.0 / 9.0;// updated 2019
	public static final double TRACTION_WHEEL_CIRCUMFERENCE = 4.0 * Math.PI;// inches
	private final TractionControl driveMotor;
	private final RotationControl turningMotor;
	private double decapitated = 1.0;
	private double tractionDeltaPathLength = 0.0;
	private double tractionPreviousPathLength = 0.0;
	public final PIDController turningPidController;
	public final PIDController turningPidController2;
	public String moduleName;
	private CANCoder angleEncoder;
	private Rotation2d lastAngle;
	private Rotation2d angleOffset;
	

	// This constructor is intended for use with the module which has an encoder on
	// the traction motor.

	public SwerveModule(int driveMotorId, int turningMotorId, int absoluteEncoderId, String name, int cancoderID, Rotation2d angleOffset) {

		moduleName = name;
		this.angleOffset = angleOffset;
        driveMotor = new TractionControl(driveMotorId);
        turningMotor = new RotationControl(turningMotorId, absoluteEncoderId);
		turningPidController = new PIDController(2, 0, 0);
		turningPidController2 = new PIDController(3, 0, 0);
        turningPidController.enableContinuousInput(-180, 180);
        driveMotor.resetEncoder();
		turningMotor.resetEncoder();
		angleEncoder = new CANCoder(cancoderID);
        configAngleEncoder();

    }

	private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
		angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		angleEncoder.configSensorDirection(false);
		angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
		//Possibly need to config sensorTimeBase
		
    }

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			driveMotor.getPositionFromIntegratedSensor(), new Rotation2d(turningMotor.getAbsoluteEncoder()));
	  }

	public double getMPS() {
		return driveMotor.getRPS() * Constants.RPS_TO_METERS_PER_SECOND;
	}

	public double getAngle() {
		
		return turningMotor.getCurrentAngle();
	}

	public double getCANCoderAngle() {
		return angleEncoder.getAbsolutePosition();
	}

	public RotationControl getTurningMotor() {
		return turningMotor;
	}

	public TractionControl getDriveMotor() {
		return driveMotor;
	}

	public SwerveModuleState getState() {
        return new SwerveModuleState(getMPS(), new Rotation2d(getAngle()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < .5
		) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
		
        driveMotor.set(state.speedMetersPerSecond / 3.83);
        setAngle(state);
		SmartDashboard.putNumber("Swerve[" + moduleName + "] angle", getAngle());
		SmartDashboard.putString("Swerve[" + moduleName + "] state", state.toString());
		
    }

	public void setAngle(SwerveModuleState desiredState) {
		Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.MAX_METERS_PER_SECOND * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        turningMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.STEERING_GEAR_RATIO));
        lastAngle = angle;
	}
	
	public void driveToDirection(double direction) {
		driveMotor.set(.5);
		turningMotor.SetAngle(direction);
	}

	public void swivelTo(double direction) {
		turningMotor.SetAngle(turningPidController.calculate(getAngle(), direction));
	}

	public void stopTraction() {
		driveMotor.set(0);
	}

    public void stop() {
        driveMotor.set(0);
		turningMotor.SetAngle(0);
    }

	public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

	public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.STEERING_GEAR_RATIO);
		turningMotor.setEncoderPosition(absolutePosition);
		
    }
   
}