// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.CollectorBalls;
import frc.robot.subsystems.DriverTrain;


public class StartJoystickArcadeDrive extends CommandBase {
private DriverTrain m_driverTrain;

  public StartJoystickArcadeDrive(DriverTrain driverTrain) {
    m_driverTrain=driverTrain;
    addRequirements(m_driverTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new CollectBall(Robot.collectorBall).execute();
    new ShootBall(Robot.shooterBall).execute();
    new ChangeAngle(Robot.changeangle).execute();
   


    double xAxis = Robot.m_oi.GetDriverRawAxis(RobotMap.STICK_Y);
    double yAxis = Robot.m_oi.GetDriverRawAxis(RobotMap.STICK_X);
    double testpov = Robot.m_oi.getJoystick().getPOV();
    Robot.driverTrain.ArcadeDrive(-xAxis, yAxis, true);
    SmartDashboard.putNumber("Joystick X Value", xAxis);
    SmartDashboard.putNumber("Joystick Y Value", yAxis);
    SmartDashboard.putNumber("Joystick POV Value", testpov);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driverTrain.ArcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}