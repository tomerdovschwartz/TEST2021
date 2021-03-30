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
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.RamseteTrajectoryCommand;
import frc.robot.commands.StartAuto;
import frc.robot.subsystems.ChangeAngleDirection;
import frc.robot.subsystems.CollectorBalls;
import frc.robot.subsystems.DriverTrain;
import frc.robot.subsystems.ShooterBalls;
import frc.robot.commands.StartJoystickArcadeDrive;

/** Add your docs here. */
public class RobotContainer {
  public static DriverTrain driverTrain = new DriverTrain();
  public static ShooterBalls shooterBall = new ShooterBalls();
  public static CollectorBalls collectorBall = new CollectorBalls();
  public static ChangeAngleDirection changeangle = new ChangeAngleDirection();
  public static OI m_oi;
  public static Object robotType;
  
   static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
   static ADXRS450_Gyro gyro = new ADXRS450_Gyro(kGyroPort);


public RobotContainer() {
    m_oi = new OI();

  }

public void onAutoInit(){
    new InstantCommand(gyro::calibrate);
  }
  
  public void onTeleopInit() {
    new StartJoystickArcadeDrive(driverTrain).schedule();
    
  }
    

public Command getAutonomousCommand(){
    TrajectoryConfig config = new TrajectoryConfig(5,1.0);
    /**NEED TO CHANGE
     * Parameters:

    maxVelocityMetersPerSecond The max velocity for the trajectory.

    maxAccelerationMetersPerSecondSq The max acceleration for the trajectory.
    */
    config.setKinematics(driverTrain.getKinematics());
    Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
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
    Trajectory trajectory2= TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(5.0,0,new Rotation2d())),config);
        
       
    String trajectoryJSON = "paths/SlalomTry.wpilib.json";
    Trajectory trajectory1 = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteTrajectoryCommand trajectoryCommand = new RamseteTrajectoryCommand(driverTrain, trajectory1);
    return new RamseteTrajectoryCommand(driverTrain, trajectory2);
}
}
