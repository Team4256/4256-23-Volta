package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

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
	private TalonFX angleMotor;
    private TalonFX driveMotor;
	public String moduleName;
	
    private CANCoder angleEncoder;
	private Rotation2d lastAngle;
	private Rotation2d angleOffset;
	
	// This constructor is intended for use with the module which has an encoder on
	// the traction motor.

	public SwerveModule(int driveMotorID, int angleMotorId, String name, int cancoderID, Rotation2d angleOffset) {

		moduleName = name;
		this.angleOffset = angleOffset;
        
        angleEncoder = new CANCoder(cancoderID);
        configAngleEncoder();

		/* Angle Motor Config */
        angleMotor = new TalonFX(angleMotorId);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorID);
        configDriveMotor();
        lastAngle = getState().angle;
	
    }

	private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
		angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		angleEncoder.configSensorDirection(false);
		angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        angleEncoder.configGetFeedbackTimeBase(50);
        //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
		//Possibly need to config sensorTimeBase
		
    }

     /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

     /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

	private void configAngleMotor(){
        angleMotor.configFactoryDefault();
        angleMotor.config_kP(0, Constants.ANGLE_KP);
        angleMotor.config_kI(0, Constants.ANGLE_KI);
        angleMotor.config_kD(0, Constants.ANGLE_KD);
        angleMotor.config_kF(0, Constants.ANGLE_KF);
        angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            Constants.ANGLE_ENABLE_CURRENT_LIMIT, 
            Constants.ANGLE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.ANGLE_PEAK_CURRENT_LIMIT, 
            Constants.ANGLE_PEAK_CURRENT_DURATION));
        angleMotor.setInverted(false);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        driveMotor.configFactoryDefault();
        driveMotor.config_kP(0, Constants.DRIVE_KP);
        driveMotor.config_kI(0, Constants.DRIVE_KI);
        driveMotor.config_kD(0, Constants.DRIVE_KD);
        driveMotor.config_kF(0, Constants.DRIVE_KF);
        driveMotor.configSupplyCurrentLimit( new SupplyCurrentLimitConfiguration(
            Constants.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.DRIVE_PEAK_CURRENT_LIMIT, 
            Constants.DRIVE_PEAK_CURRENT_DURATION));
        driveMotor.setInverted(true
        );
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

    public void setDesiredState(SwerveModuleState desiredState){
        SmartDashboard.putNumber("Swerve[" + moduleName + "] desired speed", desiredState.speedMetersPerSecond);
        if (Math.abs(desiredState.speedMetersPerSecond) <= .2) {
            stop();
            return;
        }
        

        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState);

        SmartDashboard.putNumber("Swerve[" + moduleName + "] angle", getCANCoderAngle());
		SmartDashboard.putString("Swerve[" + moduleName + "] state", desiredState.toString());
        SmartDashboard.putString("Swerve[" + moduleName + "] state angle", desiredState.angle.toString());
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

    }

    public void setSpeed (double speed) {
        driveMotor.set(ControlMode.PercentOutput, speed);
    }

	public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

	public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.STEERING_GEAR_RATIO);
		angleMotor.setSelectedSensorPosition(absolutePosition);
		
    }
   
}