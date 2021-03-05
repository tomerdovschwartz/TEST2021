// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoDrive extends SubsystemBase {
  /** Creates a new AutoDrive. */
  public AutoDrive() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

/**
 * // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriverTrain;
import frc.robot.subsystems.ShooterBalls;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;


/**
 *  The VM is configured to automatically run
 * this class, and to call the functions corresponding to each mode, as des
 * ribed in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 
public class Robot extends TimedRobot {

  RobotContainer container;
  public static DriverTrain driverTrain = new DriverTrain();
  public static ShooterBalls shooterBall = new ShooterBalls();
  private Command m_autonomousCommand;
  public static OI m_oi;
  public static Object robotType;
  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   

  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    container=new RobotContainer();
    m_oi = new OI();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
  
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. 
  
  public void disabledInit() {}


  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. 

  public void autonomousInit() {
    encoder.reset();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  final double kP = 0.5;
  final double kI = 0.5;
  final double kD = 0.1;
  final double iLimit = 1;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  /** This function is called periodically during autonomous. 

  public void autonomousPeriodic() {
    // get joystick command
    if (m_oi.joystick_controller.getRawButton(1)) {
      setpoint = 3;
    } else if (m_oi.joystick_controller.getRawButton(2)) {
      setpoint = 0;
    }

     // get sensor position
     double sensorPosition = encoder.get() * kDriveTick2Feet;

     // calculations
     double error = setpoint - sensorPosition;
     double dt = Timer.getFPGATimestamp() - lastTimestamp;
 
     if (Math.abs(error) < iLimit) {
       errorSum += error * dt;
     }
 
     double errorRate = (error - lastError) / dt;
 
     double outputSpeed = kP * error + kI * errorSum + kD * errorRate;
 
     // output to motors
    driverTrain.ArcadeDrive(outputSpeed, 0, true);

     // update last- variables
     lastTimestamp = Timer.getFPGATimestamp();
     lastError = error;
    
  }

  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. 
  
  public void teleopPeriodic() {

    CommandScheduler.getInstance().run();
  }


  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. 
  
  public void testPeriodic() {}
}

 */
