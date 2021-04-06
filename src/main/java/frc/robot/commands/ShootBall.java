// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.ShooterBalls;

public class ShootBall extends CommandBase {
  private ShooterBalls m_shooterballs;
  /** Creates a new ShootBall. */
  public ShootBall(ShooterBalls shooterballs) {
    m_shooterballs=shooterballs;
    addRequirements(m_shooterballs);
  }
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
@Override
  public void execute() {
    m_shooterballs.startShoot();
    m_shooterballs.startGrab();
  
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if((new OI().getJoystick().getRawButtonReleased(2)))
    m_shooterballs.stopShot();

    if((new OI().getJoystick().getRawButtonReleased(3)))
    m_shooterballs.stopGrab();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return (((new OI().getJoystick().getRawButtonReleased(2)))||(new OI().getJoystick().getRawButtonReleased(3)));
  }
}