// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import  edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriverTrain;


public class RamseteTrajectoryCommand extends RamseteCommand {
  public RamseteTrajectoryCommand(DriverTrain m_drivertrain, Trajectory trajectory) {
    super(
      trajectory,
      m_drivertrain::getPose,
      new RamseteController(2.0,0.7),
      m_drivertrain.getFeedForward(),
      m_drivertrain.getKinematics(), 
      m_drivertrain::getSpeeds,
      m_drivertrain.getLeftPIDController(),
      m_drivertrain.getLeftPIDController(),
      m_drivertrain::setOutputVolatage,
      m_drivertrain
      );
  }
 
  
}  
  

