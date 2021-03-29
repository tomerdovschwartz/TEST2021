// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.ChangeAngleDirection;
import frc.robot.subsystems.CollectorBalls;
import frc.robot.subsystems.DriverTrain;
import frc.robot.subsystems.ShooterBalls;



/**
 *  The VM is configured to automatically run
 * this class, and to call the functions corresponding to each mode, as des
 * ribed in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
    private ADXRS450_Gyro  gyro ;
  private Command m_autonomousCommand;
  //SendableChooser <AutoCommandInterface > chosenAuto = new SendableChooser<>();


  public static DriverTrain driverTrain = new DriverTrain();
  public static ShooterBalls shooterBall = new ShooterBalls();
  public static CollectorBalls collectorBall = new CollectorBalls();
  public static ChangeAngleDirection changeangle = new ChangeAngleDirection();

  public static OI m_oi;
  public static Object robotType;

  /** This function is called periodically during autonomous. */
 double leftslow = 0.3*12; 
 double rightslow = -0.3*12;
 double rotatespeed = 0.35*12;
 double rotatespeedslow = 0.25*12;
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our container=new RobotContainer();
   // m_autonomousCommand = container.getAutonomousCommand();

   gyro=driverTrain.getGyro();
   m_oi = new OI();


   //FOR CAMERA//
  //  new Thread(() -> {
  //   UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
  //   camera.setResolution(640, 480);

  //   CvSink cvSink = CameraServer.getInstance().getVideo();
  //   CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

  //   Mat source = new Mat();
  //   Mat output = new Mat();

  //   while(!Thread.interrupted()) {
  //     if (cvSink.grabFrame(source) == 0) {
  //       continue;
  //     }
  //     Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
  //     outputStream.putFrame(output);ַ
  //   }
  // }).start();
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
 
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      
     gyro.calibrate();

    }
  
  }
 
  @Override
  public void autonomousPeriodic() {
    
    CommandScheduler.getInstance().run();
    driverTrain.ArcadeDrive(1, 0, true);
    // if(Math.abs(gyro.getAngle())<=10){
    //   driverTrain.setOutput(leftslow-(gyro.getAngle())/15, rightslow-(gyro.getAngle())/15);
    //   }else
    //    if (Math.abs (gyro.getAngle())< 20){
    //       if(gyro.getAngle()>0){
    //               driverTrain.setOutput(leftslow, 1.1*rightslow);
    //              } else 
    //              if (gyro.getAngle()<0){
    //               driverTrain.setOutput(1.1*leftslow,rightslow);
                  
    //   }else 
    //     if(gyro.getAngle()> 0){
    //       while (gyro.getAngle() > 20 && isAutonomous()){
    //         driverTrain.setOutput(-rotatespeed, -rotatespeed);
    //       }
    //       while (gyro.getAngle() > 0 && isAutonomous()){
    //         driverTrain.setOutput(-rotatespeedslow, -rotatespeedslow);
    //       }
    //       while (gyro.getAngle() < 0 && isAutonomous()){
    //         driverTrain.setOutput(rotatespeedslow, rotatespeedslow);
    //       }
    //     }else {
    //        while (gyro.getAngle() < -20 && isAutonomous()){
    //         driverTrain.setOutput(rotatespeed, rotatespeed);
    //       }
    //       while (gyro.getAngle() < 0 && isAutonomous()){
    //         driverTrain.setOutput(rotatespeedslow, rotatespeedslow);
    //       }
    //       while (gyro.getAngle() > 0 && isAutonomous()){
    //         driverTrain.setOutput(-rotatespeedslow, -rotatespeedslow);
    //       }
    //     }
    //   }
  }
    



  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    // SmartDashboard.putNumber("angle1", gyro.getAngle());
    // SmartDashboard.putNumber("angle2", gyro.getRate());
    // SmartDashboard.putNumber("angle3", gyro.getRotation2d().getDegrees());


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
