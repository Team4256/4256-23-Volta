package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

  // Controller
  public static final double CONTROLLER_DEADBAND = 0.05;//.25
  public static final int DRIVER_CONTROLLER_ID = 0;
  public static final int GUNNER_CONTROLLER_ID = 1;
  
  // Gyro
  public static final byte GYRO_UPDATE_HZ = 50;
  public static final double GYRO_OFFSET = 0;
  public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
  public static final double BEAM_BALANACED_DRIVE_KP = .001;

  // Swerve
  // Distance between right and left wheels
  public static final double TRACK_WIDTH = Units.inchesToMeters(20.5);//done 2022
  // Distance between front and back wheels
  public static final double WHEEL_BASE = Units.inchesToMeters(25.25); //done 2022
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); // inches
  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
  );
  public static final double WHEEL_CIRCUMFERENCE = .159; //meters
  public static final double TRACTION_GEAR_RATIO = 6.752; 

  public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25; 
  public static final int ANGLE_PEAK_CURRENT_LIMIT = 40; 
  public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1; 
  public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true; 

  public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35; 
  public static final int DRIVE_PEAK_CURRENT_LIMIT = 60; 
  public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1; 
  public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true; 
  
  public static final double OPEN_LOOP_RAMP = 0.25;
  public static final double CLOSED_LOOP_RAMP = 0.0;

   /* Drive Motor PID Values */
   public static final double DRIVE_KP = 0.05; //TODO: This must be tuned to specific robot
   public static final double DRIVE_KI = 0.0;
   public static final double DRIVE_KD = 0.0;
   public static final double DRIVE_KF = 0.0;

   public static final double ANGLE_KP = 0.1; //TODO: This must be tuned to specific robot
   public static final double ANGLE_KI = 0.0;
   public static final double ANGLE_KD = 0.0;
   public static final double ANGLE_KF = 0.0;

  /**
   * Swerve #1 = 
   * Swerve #2 = 1.22
   * Swerve #3 = 4.35
   * Swerve #4 = 
   * Swerve #5 = 2.89
   * Swerve #6 = 
   * Swerve #7 = 
   * Swerve #8 = 
   * Swerve #9 = 1.00
   */

   public static final int MODULE_A_CANCODER_ID = 31; //TODO get real ID's
   public static final int MODULE_B_CANCODER_ID = 32; //TODO get real ID's
   public static final int MODULE_C_CANCODER_ID = 33; //TODO get real ID's
   public static final int MODULE_D_CANCODER_ID = 34; //TODO get real ID's
  
   public static final double STEERING_GEAR_RATIO = 12.8;


   public static final Rotation2d MODULE_A_ANGLE_OFFSET = Rotation2d.fromDegrees(55.63);
   public static final Rotation2d MODULE_B_ANGLE_OFFSET = Rotation2d.fromDegrees(278.61);
   public static final Rotation2d MODULE_C_ANGLE_OFFSET = Rotation2d.fromDegrees(1.49);
   public static final Rotation2d MODULE_D_ANGLE_OFFSET = Rotation2d.fromDegrees(97.64);

  public static final int ROTATION_MOTOR_A_ID = 11; // Front Left //all done 2022
  public static final int ROTATION_MOTOR_B_ID = 12; // Front Right
  public static final int ROTATION_MOTOR_C_ID = 13; // AFT Left
  public static final int ROTATION_MOTOR_D_ID = 14; // AFT Right
  public static final int TRACTION_MOTOR_A_ID = 21; // Front Left
  public static final int TRACTION_MOTOR_B_ID = 22; // Front Right
  public static final int TRACTION_MOTOR_C_ID = 23; // AFT Left
  public static final int TRACTION_MOTOR_D_ID = 24; // AFT Right
  //ensure swerves are on analog 0-3
  public static final int ROTATION_ENCODER_A_ID = 0; // Front Left
  public static final int ROTATION_ENCODER_B_ID = 1; // Front Right
  public static final int ROTATION_ENCODER_C_ID = 2; // Aft Left
  public static final int ROTATION_ENCODER_D_ID = 3; // Aft Right
  public static final double ABSOLUTE_ENCODER_A_TARE = 1.22; // Front Left
  public static final double ABSOLUTE_ENCODER_B_TARE = 2.89; // Front Right
  public static final double ABSOLUTE_ENCODER_C_TARE = 4.29; // Aft Left
  public static final double ABSOLUTE_ENCODER_D_TARE = 1.01; // Aft Right
  public static final double ANGLE_A_TARE = 9.074; // Front Left (angle) //new 2022, not implemented yet...
  public static final double ANGLE_B_TARE = 6.264; // Front Right
  public static final double ANGLE_C_TARE = 3.840; // Aft Left
  public static final double ANGLE_D_TARE = 6.723; // Aft Right
  
  public static final double MAX_METERS_PER_SECOND = 3.83; // Max Speed TODO
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;
  public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED =  Math.PI / 4;
  public static final double TELEOP_SPEED_LIMIT_MPS = MAX_METERS_PER_SECOND;
  public static final double TELEOP_ANGULAR_SPEED_LIMIT_RADIANS_PER_SECOND = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 2;
  public static final double MAX_ACCELERATION = 3;
  public static final double TELEOP_MAX_ANGULAR_ACCELERATION = 3;
  public static final double ENCODER_CONVERSION_TO_REVOLUTIONS_PER_SECONDS = 1 / 7.04; //gear ratio * encoder conversion *
  public static final double RPS_TO_METERS_PER_SECOND = ENCODER_CONVERSION_TO_REVOLUTIONS_PER_SECONDS * Math.PI * WHEEL_DIAMETER;
  // Automomous
  public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints( //
    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
  );

  // Elevator
  public final static int ELEVATOR_MOTOR_ID = 0;
  public final static int ELEVATOR_UPPER_LIMIT = 0;
  public final static int ELEVATOR_POSITION_HIGH = 0;
  public final static int ELEVATOR_POSITION_MID = 0;
  public final static int ELEVATOR_POSITION_LOW = 0;
  public final static double ELEVATOR_MOTOR_KP = 0;
  public final static double ELEVATOR_MOTOR_KI = 0;
  public final static double ELEVATOR_MOTOR_KD = 0;
  public final static double ELEVATOR_MOTOR_KF = 0;
  

  // Conveyor
  public static final int INTAKE_MOTOR_ID = 16;
  public static final int SHOOTER_MOTOR_ID = 26;
  public static final int CONVEYOR_MOTOR_ID = 15;
  public static final double INTAKE_MOTOR_SPEED = -0.5;//changed from 0.5
  public static final double OUTTAKE_MOTOR_SPEED = 0.5;//changed from 0.5
  public static final double CONVEYOR_MOTOR_SPEED = 0.5;//changed from 0.5
  public static final double SHOOTER_MOTOR_SPEED = 1.0;//changed from 0.8
  public static final int INTAKE_BALL_SENSOR_ID = 0;
  public static final int CONVEYOR_BALL_SENSOR_ID = 1;
  public static final int INTAKE_DOWN_CHANNEL = 4;
  public static final int INTAKE_UP_CHANNEL = 5;


  public static final double angleEncoderMinVoltage[] = {
  //   .008544921,
  //   .020751951,
  //   0.01220703,
  //   0.026855466,
  0.018,
  0.062,
  0.018,
  0.020
   };
  public static final double angleEncoderMaxVoltage[] = {
    // 4.921874496,
    // 4.921874496,
    // 4.913329475000001,
    // 4.887694812,
    4.92,
    4.91,
    4.92,
    4.94
  };
  // COMPETITION ROBOT
  //ensure swerves are on analog 0-3
  /**
   * Swerve #1 = 
   * Swerve #2 = 1.22
   * Swerve #3 = 4.35
   * Swerve #4 = 
   * Swerve #5 = 2.89
   * Swerve #6 = 
   * Swerve #7 = 
   * Swerve #8 = 
   * Swerve #9 = 1.00
   */
  public static final double angleEncoderTareVoltage[] = {
   //2022 Competition Values
    // 1.22,
    // 2.89,
    // 4.35,
    // 1.00
    1.22,
    1.08,
    1.20,
    4.25
  };

  // Aligner tolerances

  public static final double POSITION_TOLERANCE = 5.00;
  public static final double VELOCITY_TOLERANCE = 5.00;
}