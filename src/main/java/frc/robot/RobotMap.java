package frc.robot;

public class RobotMap {

    public static final int MOTOR_LEFT_1_ID = 0;
	public static final int MOTOR_LEFT_2_ID = 1;
	public static final int MOTOR_RIGHT_1_ID = 2;
    public static final int MOTOR_RIGHT_2_ID = 3;
    
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

    public static final double PATH_MAX_SPEED = 4.572;

    public static final int USONIC_PIN = 0;

    public static final int ROLLER_LEFT = 4;
    public static final int ROLLER_RIGHT = 5;
    public static final int ROLLER_PISTON = 4;

    public static final int JOYSTICK_LEFT = 0;
    public static final int JOYSTICK_RIGHT = 1;
    public static final int XBOX_CONTROLLER = 2;

    public static final int LIGHT_VOLTAGE = 0;

    public static final int RIGHT_RAMP_FORWARD = 0;
    public static final int RIGHT_RAMP_REVERSE = 1;
    public static final int LEFT_RAMP_FORWARD = 2;
    public static final int LEFT_RAMP_REVERSE = 3;
    public static final int RAMP_WINCH = 11;

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
    public static final int GANI_POWER = 4;
}
