package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class RobotMap {
    /**
	 * How many sensor units per rotation.
	 * Using CTRE Magnetic Encoder.
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
    public final static int kSensorUnitsPerRotation = 4096;

	/**
	 * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation.
	 * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
	 */
	public final static double kTurnTravelUnitsPerRotation = 3600;
	
	/**
	 * Empirically measure what the difference between encoders per 360'
	 * Drive the robot in clockwise rotations and measure the units per rotation.
	 * Drive the robot in counter clockwise rotations and measure the units per rotation.
	 * Take the average of the two.
	 */
	public final static int kEncoderUnitsPerRotation = 51711;
    
    public final static double RADIUS_WHELL=0.1;
    public final static int MAX_VOLT=12;

    public static final int DRIVER_CONTROLLER = 0;
    public static final int STICK_X = 0;
	public static final int STICK_Y = 1;
	public static final int STICK_Z = 2;
	public static final int THOROTTLE = 3;

	public static final int DRIVE_RIGHT_MASTER = 2;
    public static final int DRIVE_RIGHT_FOLLOW_ONE = 2;
    public static final int DRIVE_RIGHT_FOLLOW_TWO = 3;

    public static final int DRIVE_LEFT_MASTER = 0;
    public static final int DRIVE_LEFT_FOLLOW_ONE = 0;
    public static final int DRIVE_LEFT_FOLLOW_TWO = 1;
    

    public static final int GRAB_MASTER=8;
    public static final int SHOOT_MASTER=9;
    

    public static final int COLLECT_MASTER=6;

    public static final int CHANGEDIRECTION_MASTER=7;
    
    public static final double PATH_MAX_SPEED = 4.572;


    public static final int JOYSTICK_LEFT = 0;
    public static final int JOYSTICK_RIGHT = 1;

    public static final int LIGHT_VOLTAGE = 0;

    public static final int RAMP_CLAW_FORWARD = 5;
    public static final int RAMP_CLAW_REVERSE = 6;

    public static final int LIFT_MOTOR = 6;
    public static final int LIFT_HALL_TOP = 0;
    public static final int LIFT_HALL_BOT = 1;

    private static final double ENCODER_PULSES_PER_REVOLUTION = 256.0D;
    public static final double WHEEL_TURNS_PER_ENCODER_TURN = 1.0 / 2.975;
    public static final double INCHES_PER_WHEEL_TURN = 6.0 * Math.PI;

    public static final double INCHES_PER_COUNT = (1.0 / ENCODER_PULSES_PER_REVOLUTION) * WHEEL_TURNS_PER_ENCODER_TURN * INCHES_PER_WHEEL_TURN;
    public static final double COUNTS_PER_INCH = 1.0 / INCHES_PER_COUNT;

    public static final double FEET_PER_COUNT = (1.0 / 12.0) * INCHES_PER_COUNT;
    public static final double COUNTS_PER_FOOT = COUNTS_PER_INCH * 12.0;

    public static final double COUNTS_PER_METER = COUNTS_PER_INCH * (1 / 2.54) * 100.0;
    public static final double METERS_PER_COUNT = 1.0 / COUNTS_PER_METER;

    public static final DifferentialDriveKinematics kinematics = new  DifferentialDriveKinematics(0.5);
    public static final PIDController leftPIDController= new PIDController(1.0, 1, 0); 
    public static final  PIDController rightPIDController= new PIDController(1.0, 1, 0); 
    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 2);

    public static boolean GrabWorkKey=false;
    public static boolean ShotWorkKey=false;
    public static boolean CollectWorkKey=false;

    //-----------------------------Galactic Search A RED---------------------------------------------//
    public static final double GalacticSearch_PathA_RED_Xcord [] = {
        
    };
    public static final double GalacticSearch_PathA_RED_Ycord [] ={
     
    };
    public static final int GalacticSearch_PathA_RED_Collect [] ={
     
    };
    public static final int GalacticSearch_PathA_RED_Grab [] ={
     
    };
    //-----------------------------Galactic Search A BLUE---------------------------------------------//
    public static final double GalacticSearch_PathA_BLUE_Xcord [] = {
        
    };
    public static final double GalacticSearch_PathA_BLUE_Ycord [] ={
     
    };
    public static final int GalacticSearch_PathA_BLUE_Collect [] ={
     
    };
    public static final int GalacticSearch_PathA_BLUE_Grab [] ={
     
    };
    
    
    //-----------------------------Galactic Search B RED---------------------------------------------//
    public static final double GalacticSearch_PathB_RED_Xcord [] = {
        
    };
    public static final double GalacticSearch_PathB_RED_Ycord [] ={
     
    };
    public static final int GalacticSearch_PathB_RED_Collect [] ={
     
    };
    public static final int GalacticSearch_PathB_RED_Grab [] ={
     
    };
    //-----------------------------Galactic Search B BLUE---------------------------------------------//
    public static final double GalacticSearch_PathB_BLUE_Xcord [] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.300000012, 0.600000083, 0.600000083, 0.300000012, 7.45E-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        
    };
    public static final double GalacticSearch_PathB_BLUE_Ycord [] ={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.150000006, -0.450000048, -0.750000119, -1, -1, -1, -1, -0.699999928, -0.399999857, -0.099999845, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.25, -0.550000072, -0.850000143, -0.899999976, -0.599999905, -0.299999833, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.200000003, -0.50000006, -0.800000131, -1, -0.699999928, -0.399999857, -0.099999845, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

     
    };
  
    public static final int GalacticSearch_PathB_BLUE_Collect [] ={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

     
    };
    public static final int GalacticSearch_PathB_BLUE_Grab [] ={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

     
    };
    //-----------------------------AutoNav Challenge Barrel Racing---------------------------------------------//
    public static final double AutoNavChallenge_PathBarrelRacing_Xcord [] = {
        
    };
    public static final double AutoNavChallenge_PathBarrelRacing_Ycord [] = {
        
    };
    //-----------------------------AutoNav Challenge Slalom---------------------------------------------//
    public static final double AutoNavChallenge_PathSlalom_Xcord [] = {
        
    };
    public static final double AutoNavChallenge_PathSlalom_Ycord [] = {
        
        
    };
    //-----------------------------AutoNav Challenge Barrel Bounce---------------------------------------------//
    public static final double AutoNavChallenge_PathBounce_Xcord [] = {
        
    };
    public static final double AutoNavChallenge_PathBounce_Ycord [] = {
        
    };
    
    
}