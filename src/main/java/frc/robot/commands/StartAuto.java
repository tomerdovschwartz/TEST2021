// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.sql.Time;
import java.util.Arrays;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriverTrain;

public class StartAuto extends CommandBase {
  private DriverTrain m_drivertrain;
  private ADXRS450_Gyro  gyro ;
  double striaghtspeed = 0.18*12; 
  double rightslow = 0.18*12;
  double rotatespeed = 0.55*12;
  double rotatespeedslow = 0.35*12;
  double anglegyro = 0.0;
  private double startTime;
  

  public StartAuto(DriverTrain drivertrain) {
    m_drivertrain=drivertrain;
    gyro=m_drivertrain.getGyro();
    addRequirements(m_drivertrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    gyro.calibrate();
    
  }


  @Override
public void execute() 
{
  m_drivertrain.ArcadeDrive(0.4, 0, false);

}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
