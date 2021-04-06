// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  

  // public StartAuto(DriverTrain drivertrain) {
  //   m_drivertrain=drivertrain;
  //   gyro=m_drivertrain.getGyro();
  //   addRequirements(m_drivertrain);
  // }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {
  //   startTime = Timer.getFPGATimestamp();
  //   gyro.calibrate();
    
  // }


  @Override
public void execute() 
{


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
