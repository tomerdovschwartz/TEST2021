// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.CollectorBalls;



public class CollectBall extends CommandBase {
  private CollectorBalls m_collectorballs;
  /** Creates a new ShootBall. */
  public CollectBall(CollectorBalls collectorballs) {
    m_collectorballs=collectorballs;
    addRequirements(m_collectorballs);
  }
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_collectorballs.startCollect();
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collectorballs.stopCollect();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !(new OI().getJoystick().getRawButton(1));
  }

  
}
