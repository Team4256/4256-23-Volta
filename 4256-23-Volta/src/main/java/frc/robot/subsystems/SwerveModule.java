package frc.robot.subsystems;

import java.time.chrono.ThaiBuddhistChronology;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Conversions;
import frc.robot.Robot;

public final class SwerveModule {
	public static final double ROTATOR_GEAR_RATIO = 1.0;
	public static final double TRACTION_GEAR_RATIO = 52.0 / 9.0;// updated 2019
	public static final double TRACTION_WHEEL_CIRCUMFERENCE = 4.0 * Math.PI;// inches
	private TalonFX angleMotor;
    private TalonFX driveMotor;
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

	public SwerveModule(int driveMotorID, int angleMotorId, int absoluteEncoderId, String name, int cancoderID, Rotation2d angleOffset) {

		moduleName = name;
		this.angleOffset = angleOffset;
		turningPidController = new PIDController(2, 0, 0);
		turningPidController2 = new PIDController(3, 0, 0);
        turningPidController.enableContinuousInput(-180, 180);
        
		/* Angle Motor Config */
        angleMotor = new TalonFX(angleMotorId);
        //configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorID);
        //configDriveMotor();

        lastAngle = getState().angle;
		
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

	private void configAngleMotor(){
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(CTREConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(true);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(CTREConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setSelectedSensorPosition(0);


    }


	  public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.WHEEL_CIRCUMFERENCE, Constants.TRACTION_GEAR_RATIO), 
            getAngle()
        );
    }

	private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.STEERING_GEAR_RATIO));
    }

	public double getCANCoderAngle() {
		return angleEncoder.getAbsolutePosition();
	}

	public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.WHEEL_CIRCUMFERENCE, Constants.TRACTION_GEAR_RATIO), 
            getAngle()
        ); 
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < .5
		) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
		
        setSpeed(state);
        setAngle(state);
		SmartDashboard.putNumber("Swerve[" + moduleName + "] angle", getCANCoderAngle());
		SmartDashboard.putString("Swerve[" + moduleName + "] state", state.toString());
		
    }

	public void setAngle(SwerveModuleState desiredState) {
		Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.MAX_METERS_PER_SECOND * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.STEERING_GEAR_RATIO));
        lastAngle = angle;
	}

	public void setAngleDegrees(double angle) {
		angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.STEERING_GEAR_RATIO));
		
	}
	
	private void setSpeed(SwerveModuleState desiredState){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.MAX_METERS_PER_SECOND;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
			SmartDashboard.putNumber("Desired speed", percentOutput);
    }

	public void stopTraction() {
		driveMotor.set(ControlMode.PercentOutput, 0);
	}

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput,0);
		setAngleDegrees(0);
		angleMotor.set(ControlMode.Position, ROTATOR_GEAR_RATIO);
    }

	public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

	public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.STEERING_GEAR_RATIO);
		angleMotor.setSelectedSensorPosition(absolutePosition);
		
    }
   
}