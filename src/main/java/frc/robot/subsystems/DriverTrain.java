// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.core.sym.Name;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import java.lang.Math;


public class DriverTrain extends SubsystemBase {
  /**
   * Master Talon SRX, Left side.
   */
  private TalonSRX driveLeftMaster;
  /**
   * Follower Talon SRX, left side.
   */
  private TalonSRX driveLeftFollowOne;
  private TalonSRX driveLeftFollowTwo;

  /**
   * Master Talon SRX, right side.
   */
  private TalonSRX driveRightMaster;
  /**
   * Follower Talon SRX, right side.
   */
  private TalonSRX driveRightFollowOne;
  private TalonSRX driveRightFollowTwo;
  
   static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
   ADXRS450_Gyro gyro;
  
   DifferentialDriveKinematics kinematics = new  DifferentialDriveKinematics(0.52);
   DifferentialDriveOdometry  odometry ;
   private Pose2d pose;
   
  PIDController leftPIDController= new PIDController(8.5, 0, 0); 
  PIDController rightPIDController= new PIDController(8.5, 0, 0); 

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.22,1.98,0.2);;

  

/** Creates a new DriverTrain. */
  public DriverTrain() {
    
        double talon_P = 1.0D;
        this.driveLeftMaster = new TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftMaster.setNeutralMode(NeutralMode.Brake);
        this.driveLeftMaster.config_kP(0, talon_P, 10);
        this.driveLeftMaster.config_kF(0, 1 / (RobotMap.PATH_MAX_SPEED * RobotMap.COUNTS_PER_METER * 4.0 * (1.0 / 10.0)), 10);

        
        this.driveLeftFollowOne = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_ONE);
        this.driveLeftFollowOne.set(ControlMode.Follower, RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftFollowOne.setNeutralMode(NeutralMode.Brake);

        this.driveLeftFollowTwo = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_TWO);
        this.driveLeftFollowTwo.set(ControlMode.Follower, RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftFollowTwo.setNeutralMode(NeutralMode.Brake);

        this.driveRightMaster = new TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightMaster.setNeutralMode(NeutralMode.Brake);
        this.driveRightMaster.config_kP(0, talon_P, 10);
        this.driveRightMaster.config_kF(0, 1 / (RobotMap.PATH_MAX_SPEED * RobotMap.COUNTS_PER_METER * 4.0 * (1.0 / 10.0)), 10);
        this.driveRightMaster.setInverted(true);
        this.driveRightMaster.setSensorPhase(true);

        this.driveRightFollowOne = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_ONE);
        this.driveRightFollowOne.set(ControlMode.Follower, RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightFollowOne.setNeutralMode(NeutralMode.Brake);
        this.driveRightFollowOne.setInverted(true);

        this.driveRightFollowTwo = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_TWO);
        this.driveRightFollowTwo.set(ControlMode.Follower, RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightFollowTwo.setNeutralMode(NeutralMode.Brake);
        this.driveRightFollowTwo.setInverted(true);    
        
          // Configure the motors
          for(TalonSRX fx : new TalonSRX[] {driveLeftMaster,driveLeftFollowOne,driveLeftFollowTwo, driveRightMaster,driveRightFollowOne,driveRightFollowTwo}){
            //Reset settings for safety
           // fx.configFactoryDefault();

            //Sets voltage compensation to 12, used for percent output
            fx.configVoltageCompSaturation(12);
            fx.enableVoltageCompensation(true);

            //Setting just in case
            fx.configNominalOutputForward(0);
            fx.configNominalOutputReverse(0);
            fx.configPeakOutputForward(1);
            fx.configPeakOutputReverse(-1);
            fx.configPeakCurrentLimit(30); // don't activate current limit until current exceeds 30 A ...
            fx.configPeakCurrentDuration(100); // ... for at least 100 ms
            fx.configContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
            fx.enableCurrentLimit(true);

            fx.configOpenloopRamp(0.1);

            //Setting deadband(area required to start moving the motor) to 1%
            fx.configNeutralDeadband(0.01);

            //Either using the integrated Falcon sensor or an external one, will change if needed
            fx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        }

        gyro = new ADXRS450_Gyro(kGyroPort);
        gyro.calibrate();
        
        Rotation2d initialHeading = new Rotation2d(gyro.getAngle());
        pose=new Pose2d(0,0,initialHeading);
        odometry = new DifferentialDriveOdometry(initialHeading, pose);
     

  }
    

    @Override
    public void periodic() {
     pose=odometry.update(getHeading(),getSpeeds().leftMetersPerSecond,getSpeeds().rightMetersPerSecond);
    //  SmartDashboard.putNumber("getPose() X:", getPose().getX());
    //  SmartDashboard.putNumber("getPose() Y:", getPose().getY());
    //  SmartDashboard.putNumber("getSpeeds() leftMetersPerSecond", getSpeeds().leftMetersPerSecond);
    //  SmartDashboard.putNumber("getSpeeds() rightMetersPerSecondMetersPerSecond", getSpeeds().rightMetersPerSecond);
    
    }
    
    
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    
    
    public DifferentialDriveWheelSpeeds getSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(
          
            driveLeftMaster.getMotorOutputPercent()*3.0,
            driveRightMaster.getMotorOutputPercent()*3.0
            );
 
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
        
        return odometry.getPoseMeters();
    }
   

    public SimpleMotorFeedforward getFeedForward(){
        return feedforward;
    }

    public  ADXRS450_Gyro getGyro(){
        return gyro;
    }

    public void setOutputVolatage(double leftVolts,double rightVolts){
        leftVolts = FixVolt(leftVolts);
        rightVolts = FixVolt(rightVolts);
        System.out.println("Left Volts: leftVolts "+leftVolts+"Right Volts: rightVolts "+rightVolts);
        driveLeftMaster.set(ControlMode.PercentOutput, leftVolts/12);
        driveRightMaster.set(ControlMode.PercentOutput, rightVolts/12);
    }

    public double FixVolt(double Volt){
        if(Volt<6)
        Volt=0;

        Volt=Math.round(Math.abs(Volt));
        
        if (Volt>12)
        Volt=12;

        return Volt;
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

  public void DriveByRecorde (double [] cordinateX, double [] cordinateY,String NamePath,Boolean NEEDBALL){
    double startTimedrive ;
    int [] GrabON=null ;
    int [] CollectON=null ;

    switch(NamePath){
        case "GalacticSearch_PathA_RED":
        GrabON=RobotMap.GalacticSearch_PathA_RED_Grab;
        CollectON=RobotMap.GalacticSearch_PathA_RED_Collect;

        case "GalacticSearch_PathA_BLUE":
         GrabON=RobotMap.GalacticSearch_PathA_BLUE_Grab;
        CollectON=RobotMap.GalacticSearch_PathA_BLUE_Collect;
  
        case "GalacticSearch_PathB_RED":
        GrabON=RobotMap.GalacticSearch_PathB_RED_Grab;
        CollectON=RobotMap.GalacticSearch_PathB_RED_Collect;
  
        case "GalacticSearch_PathB_BLUE":
        GrabON=RobotMap.GalacticSearch_PathB_BLUE_Grab;
        CollectON=RobotMap.GalacticSearch_PathB_BLUE_Collect;
    }
  
    for (int i=0; i< cordinateX.length; i++){
        startTimedrive = Timer.getFPGATimestamp();
            ArcadeDrive(-cordinateY[i], cordinateX[i], true);
            if(NEEDBALL)
            {
                if(GrabON[i]==1)
                new ShooterBalls().startGrab();
                else
                new ShooterBalls().stopGrab();

                if(CollectON[i]==1)
                new CollectorBalls().startCollect();
                else
                new CollectorBalls().stopCollect();
                
            }
            while((Timer.getFPGATimestamp()-startTimedrive)<0.15){
                
            }
           SmartDashboard.putNumber("PLACE", i);
        }
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
