// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class ShooterBalls extends SubsystemBase {
  private VictorSPX gSpx_GrabMaster = null;
  private VictorSPX gSpx_ShootMaster = null;

  public ShooterBalls() {
  this.gSpx_ShootMaster = new VictorSPX(RobotMap.SHOOT_MASTER);
  this.gSpx_ShootMaster.setNeutralMode(NeutralMode.Brake);

  this.gSpx_GrabMaster = new VictorSPX(RobotMap.GRAB_MASTER);
  this.gSpx_GrabMaster.setNeutralMode(NeutralMode.Brake);

  }
  public void startShoot(){
      SmartDashboard.putBoolean("Shoot Work", !(RobotMap.ShotWorkKey));
    gSpx_ShootMaster.set(ControlMode.PercentOutput, 1); 
    
  }
  public void startGrab(){
      SmartDashboard.putBoolean("Grab Work", !(RobotMap.GrabWorkKey));
      gSpx_GrabMaster.set(ControlMode.PercentOutput, 0.8); 
    
  }

 
  public void stopGrab() {
    SmartDashboard.putBoolean("Grab Work", RobotMap.GrabWorkKey);
    gSpx_GrabMaster.set(ControlMode.PercentOutput, 0);
  }
  public void stopShot() {
    SmartDashboard.putBoolean("Shoot Work", RobotMap.ShotWorkKey);
    gSpx_ShootMaster.set(ControlMode.PercentOutput, 0); 
  }
}
