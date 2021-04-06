// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.ChangeAngleDirection;

public class ChangeAngle extends CommandBase {
  private ChangeAngleDirection m_changeangkedirection;
  /** Creates a new ShootBall. */
  public ChangeAngle(ChangeAngleDirection changeangkedirection) {
    m_changeangkedirection=changeangkedirection;
    addRequirements(m_changeangkedirection);
  }
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_changeangkedirection.startChangeDirection();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_changeangkedirection.stopChangeAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((new OI().getJoystick().getPOV())==-1);
  
}
}
