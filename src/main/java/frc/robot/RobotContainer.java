// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChangeAngle;
import frc.robot.commands.CollectBall;
import frc.robot.commands.GrabBall;
import frc.robot.commands.RamseteTrajectoryCommand;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.ChangeAngleDirection;
import frc.robot.subsystems.CollectorBalls;
import frc.robot.subsystems.DriverTrain;
import frc.robot.subsystems.ShooterBalls;
import frc.robot.commands.StartJoystickArcadeDrive;

/** Add your docs here. */
public class RobotContainer {
  private final DriverTrain driverTrain = new DriverTrain();
  private final ShooterBalls shooterBall = new ShooterBalls();
  private final CollectorBalls collectorBall = new CollectorBalls();
  private final ChangeAngleDirection changeangle = new ChangeAngleDirection();
  private final OI m_oi;  
   private final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
   private final ADXRS450_Gyro gyro;
   private final SendableChooser<AutoType> m_autoChooser;

   public enum AutoType{
     DO_NOTHING, 
     GalacticSearch_PathA_RED, 
     GalacticSearch_PathA_BLUE, 
     GalacticSearch_PathB_RED, 
     GalacticSearch_PathB_BLUE,
     AutoNavChallenge_PathBarrelRacing, 
     AutoNavChallenge_PathSlalom, 
     AutoNavChallenge_PathBounce;
     
   }
 

public RobotContainer() {

  m_autoChooser = new SendableChooser<>();
  m_autoChooser.setDefaultOption("Do Nothing", AutoType.DO_NOTHING);
  m_autoChooser.addOption("GalacticSearch PathA RED", AutoType.GalacticSearch_PathA_RED);
  m_autoChooser.addOption("GalacticSearch PathA BLUE", AutoType.GalacticSearch_PathA_BLUE);
  m_autoChooser.addOption("GalacticSearch PathB RED", AutoType. GalacticSearch_PathB_RED);
  m_autoChooser.addOption("GalacticSearch PathB BLUE", AutoType.GalacticSearch_PathB_BLUE);
  m_autoChooser.addOption("AutoNavChallenge Path BarrelRacing", AutoType.AutoNavChallenge_PathBarrelRacing);
  m_autoChooser.addOption("AutoNavChallenge Path Slalom", AutoType. AutoNavChallenge_PathSlalom);
  m_autoChooser.addOption("AutoNavChalleng Path Bounce", AutoType. AutoNavChallenge_PathBounce);
  SmartDashboard.putData(m_autoChooser);

  gyro = new ADXRS450_Gyro(kGyroPort);
  m_oi= new OI();
  configureButtonBindings();
  driverTrain.setDefaultCommand(new StartJoystickArcadeDrive(driverTrain));

}
public void onAutoInit(){
    new InstantCommand(gyro::calibrate);
  }
 

  public void onTeleopInit() {
    SmartDashboard.putBoolean("Grab Work", RobotMap.GrabWorkKey);
    SmartDashboard.putBoolean("Collect Work", RobotMap.CollectWorkKey);
    new StartJoystickArcadeDrive(driverTrain).schedule();
    
    
  }

  private void configureButtonBindings() {

    configureButtonShoot();
    configureButtonCollect();
    configureButtonGrab();
    configureButtonChangeAngle();


  }

  public void  configureButtonCollect(){
    m_oi.button1.toggleWhenPressed(new CollectBall(collectorBall));
    
  }

  public void  configureButtonShoot(){
    m_oi.button3.toggleWhenPressed(new ShootBall(shooterBall));
  }
  
  public void  configureButtonGrab(){
    m_oi.button2.toggleWhenPressed(new GrabBall(shooterBall));
    
  }
  
  public void  configureButtonChangeAngle(){
    m_oi.povbutton1.toggleWhenPressed(new ChangeAngle(changeangle));
    m_oi.povbutton2.toggleWhenPressed(new ChangeAngle(changeangle));
  }
public void getAutonomousCommand(){
    switch(m_autoChooser.getSelected()){
      case DO_NOTHING:
       new WaitCommand(15);

      case GalacticSearch_PathA_RED:
      driverTrain.DriveByRecorde(RobotMap.GalacticSearch_PathA_RED_Xcord, RobotMap.GalacticSearch_PathA_RED_Ycord,"GalacticSearch_PathA_RED",true);

      case GalacticSearch_PathA_BLUE:
      driverTrain.DriveByRecorde(RobotMap.GalacticSearch_PathA_BLUE_Xcord, RobotMap.GalacticSearch_PathA_BLUE_Ycord,"GalacticSearch_PathA_BLUE",true);

      case GalacticSearch_PathB_RED:
      driverTrain.DriveByRecorde(RobotMap.GalacticSearch_PathB_RED_Xcord, RobotMap.GalacticSearch_PathB_RED_Ycord,"GalacticSearch_PathB_RED",true);

      case GalacticSearch_PathB_BLUE:
      driverTrain.DriveByRecorde(RobotMap.GalacticSearch_PathB_BLUE_Xcord, RobotMap.GalacticSearch_PathB_BLUE_Ycord,"GalacticSearch_PathB_BLUE",true);

      case AutoNavChallenge_PathBarrelRacing:
      driverTrain.DriveByRecorde(RobotMap.AutoNavChallenge_PathBarrelRacing_Xcord, RobotMap.AutoNavChallenge_PathBarrelRacing_Ycord,"AutoNavChallenge_PathBarrelRacing",false);

      case AutoNavChallenge_PathSlalom:
      driverTrain.DriveByRecorde(RobotMap.AutoNavChallenge_PathSlalom_Xcord, RobotMap.AutoNavChallenge_PathSlalom_Ycord,"AutoNavChallenge_PathSlalom",false);
      
      case AutoNavChallenge_PathBounce:
      driverTrain.DriveByRecorde(RobotMap.AutoNavChallenge_PathBounce_Xcord, RobotMap.AutoNavChallenge_PathBounce_Ycord,"AutoNavChallenge_PathBounce",false);

      default:
        new WaitCommand(15);
    }
    
}
}










/*

    TrajectoryConfig config = new TrajectoryConfig(3.0,3.0);
    config.setKinematics(driverTrain.getKinematics());

    Trajectory S_Path = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );       
    Trajectory Straight_Path = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(5.0,0,new Rotation2d())),config);
        
       
    String trajectoryJSON = "output/output/Unnamed.wpilib.json";
    Trajectory Mission_Path = new Trajectory();

    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Mission_Path = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
*/