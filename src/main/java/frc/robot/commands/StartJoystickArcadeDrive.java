// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
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
    double xAxis = new OI().GetDriverRawAxis(RobotMap.STICK_Y);
    double yAxis = new OI().GetDriverRawAxis(RobotMap.STICK_X);
    double testpov = new OI().getJoystick().getPOV();
    m_driverTrain.ArcadeDrive(-xAxis, yAxis, true);
    SmartDashboard.putNumber("Joystick X Value", xAxis);
    SmartDashboard.putNumber("Joystick Y Value", yAxis);
    SmartDashboard.putNumber("Joystick POV Value", testpov);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driverTrain.ArcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}