// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriverTrain;

/** Add your docs here. */
public class RobotContainer {

private DriverTrain drive;
    
public Command getAutonomousCommand(){
    TrajectoryConfig config = new TrajectoryConfig(3.0,1.0);
    /**NEED TO CHANGE
     * Parameters:

    maxVelocityMetersPerSecond The max velocity for the trajectory.

    maxAccelerationMetersPerSecondSq The max acceleration for the trajectory.
    */
    config.setKinematics(drive.getKinematics());
    Trajectory trajectory= TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(2.0,0,new Rotation2d())),config);
        
    String trajectoryJSON = "paths/SlalomTry.wpilib.json";
    Trajectory trajectory1 = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    RamseteCommand command=new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(2.0,0.7),
        drive.getFeedForward(),
        drive.getKinematics(), 
        drive::getSpeeds,
        drive.getLeftPIDController(),
        drive.getLeftPIDController(),
        drive::setOutput,
        drive);

        return command;
}
}
