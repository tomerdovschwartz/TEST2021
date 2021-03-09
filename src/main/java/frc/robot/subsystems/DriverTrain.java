// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;
import java.lang.Math;

public class DriverTrain extends SubsystemBase {
  /**
   * Master Talon SRX, Left side.
   */
  private TalonSRX driveLeftMaster;
  private MotionProfileStatus leftMpStatus;
  /**
   * Follower Talon SRX, left side.
   */
  private TalonSRX driveLeftFollowOne;
  private TalonSRX driveLeftFollowTwo;

  /**
   * Master Talon SRX, right side.
   */
  private TalonSRX driveRightMaster;
  private MotionProfileStatus rightMpStatus;
  /**
   * Follower Talon SRX, right side.
   */
  private TalonSRX driveRightFollowOne;
  private TalonSRX driveRightFollowTwo;

 // Gyro gyro = new ADXRS450_Gyro (SPI.Port.kMXP);
AnalogGyro gyro = new AnalogGyro(1);
  DifferentialDriveKinematics kinematics = new  DifferentialDriveKinematics(0.5);
  DifferentialDriveOdometry  odometry = new DifferentialDriveOdometry (getHeading());

  Pose2d pose;
  
  PIDController leftPIDController= new PIDController(1.0, 0, 0); 
  PIDController rightPIDController= new PIDController(1.0, 0, 0); 
/**NEED TO CHANGE!! with frc-characterization
 Parameters:
    kp The proportional coefficient.
    ki The integral coefficient.
    kd The derivative coefficient
 */

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 2);
/**NEED TO CHANGE
 * Parameters:
    ks The static gain.  
    kv The velocity gain.
 */
  

/** Creates a new DriverTrain. */
  public DriverTrain() {

        double talon_P = 1.0D;
        this.driveLeftMaster = new TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftMaster.setNeutralMode(NeutralMode.Brake);
        this.driveLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        this.driveLeftMaster.config_kP(0, talon_P, 10);
        this.driveLeftMaster.config_kF(0, 1 / (RobotMap.PATH_MAX_SPEED * RobotMap.COUNTS_PER_METER * 4.0 * (1.0 / 10.0)), 10);

        this.driveLeftMaster.clearMotionProfileTrajectories();
        this.driveLeftMaster.changeMotionControlFramePeriod(25);
        this.leftMpStatus = new MotionProfileStatus();

        this.driveLeftFollowOne = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_ONE);
        this.driveLeftFollowOne.set(ControlMode.Follower, RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftFollowOne.setNeutralMode(NeutralMode.Brake);

        this.driveLeftFollowTwo = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_TWO);
        this.driveLeftFollowTwo.set(ControlMode.Follower, RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftFollowTwo.setNeutralMode(NeutralMode.Brake);

        this.driveRightMaster = new TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightMaster.setNeutralMode(NeutralMode.Brake);
        this.driveRightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        this.driveRightMaster.config_kP(0, talon_P, 10);
        this.driveRightMaster.config_kF(0, 1 / (RobotMap.PATH_MAX_SPEED * RobotMap.COUNTS_PER_METER * 4.0 * (1.0 / 10.0)), 10);
        this.driveRightMaster.setInverted(true);
        this.driveRightMaster.setSensorPhase(true);

        this.driveRightMaster.clearMotionProfileTrajectories();
        this.driveRightMaster.changeMotionControlFramePeriod(25);
        this.rightMpStatus = new MotionProfileStatus();

        this.driveRightFollowOne = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_ONE);
        this.driveRightFollowOne.set(ControlMode.Follower, RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightFollowOne.setNeutralMode(NeutralMode.Brake);
        this.driveRightFollowOne.setInverted(true);

        this.driveRightFollowTwo = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_TWO);
        this.driveRightFollowTwo.set(ControlMode.Follower, RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightFollowTwo.setNeutralMode(NeutralMode.Brake);
        this.driveRightFollowTwo.setInverted(true);
  }
  
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }
    
    public DifferentialDriveWheelSpeeds getSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(
            driveLeftMaster.getSelectedSensorVelocity()*RobotMap.RADIUS_WHELL*2*Math.PI*10/RobotMap.kSensorUnitsPerRotation,
            driveRightMaster.getSelectedSensorVelocity()*RobotMap.RADIUS_WHELL*2*Math.PI*10/RobotMap.kSensorUnitsPerRotation
            );
    /**
     * VelocitySensor=100[units/ms]
     * 100[ms]=0.1[sec]=1/600[min]
     * RPM[round/minute]=(VelocitySensor*600)/ kSensorUnitsPerRotation
     * w[angular velocity rad/s]=(2*pi*RPM)/60
     * v[linear velocity m/s]=w*WheelRadius
    */
    }

    public PIDController getLeftPIDController(){
        return leftPIDController;
    }
    public PIDController getRightPIDController(){
        return rightPIDController;
    }

    public DifferentialDriveKinematics getKinematics (){
        return kinematics;
    }

    public Pose2d getPose(){
        return pose;
    }

    public SimpleMotorFeedforward getFeedForward(){
        return feedforward;
    }

    public void setOutput(double leftVolts,double rightVolts){
        driveLeftMaster.set(ControlMode.PercentOutput, leftVolts/RobotMap.MAX_VOLT);
        driveRightMaster.set(ControlMode.PercentOutput,rightVolts/RobotMap.MAX_VOLT);
    }

    @Override
    public void periodic() {
        pose=odometry.update(getHeading(),getSpeeds().leftMetersPerSecond,getSpeeds().rightMetersPerSecond);
        setDefaultCommand(new TankDrive());
       
    }

  
  

  public void ArcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
    xSpeed = limit(xSpeed, 1);
    xSpeed = applyDeadband(xSpeed, 0.1);
    zRotation = limit(zRotation, 1);
    zRotation = applyDeadband(zRotation, 0.1);


    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squaredInputs) {
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
        // First quadrant, else second quadrant
        if (zRotation >= 0.0) {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
        } else {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
        }
    } else {
        // Third quadrant, else fourth quadrant
        if (zRotation >= 0.0) {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
        } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
        }
    }

    driveLeftMaster.set(ControlMode.PercentOutput, limit(leftMotorOutput, 1));
    driveRightMaster.set(ControlMode.PercentOutput, limit(rightMotorOutput, 1));
  }

  public static double limit(double num, double bound) {
    bound = Math.abs(bound);
    if (num > bound) {
        return bound;
    }
    if (num < -bound) {
        return -bound;
    }
    return num;
  }

  public static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
        if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
        } else {
            return (value + deadband) / (1.0 - deadband);
        }
    } else {
        return 0.0;
    }
  }
}
