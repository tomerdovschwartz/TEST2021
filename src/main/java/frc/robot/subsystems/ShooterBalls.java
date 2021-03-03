// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterBalls extends SubsystemBase {
  VictorSPX gSpx_ShootMaster=null;
  VictorSPX gSpx_ShootFollowOne = null;
  VictorSPX gSpx_ShootFollowTwo = null;

  public ShooterBalls() {
  this.gSpx_ShootMaster = new VictorSPX(RobotMap.SHOOT_MASTER);
  this.gSpx_ShootMaster.setNeutralMode(NeutralMode.Brake);

  this.gSpx_ShootFollowOne = new VictorSPX(RobotMap.SHOOT_FOLLOW_ONE);
  this.gSpx_ShootFollowOne.set(ControlMode.Follower, RobotMap.SHOOT_FOLLOW_ONE);
  this.gSpx_ShootFollowOne.setNeutralMode(NeutralMode.Brake);

  this.gSpx_ShootFollowTwo = new VictorSPX(RobotMap.SHOOT_FOLLOW_TWO);
  this.gSpx_ShootFollowTwo.set(ControlMode.Follower,RobotMap.SHOOT_MASTER);
  this.gSpx_ShootFollowTwo.setNeutralMode(NeutralMode.Brake);

  }
  public void startShoot(boolean on)
  {
    if (on){
    gSpx_ShootMaster.set(ControlMode.PercentOutput, 0.5);
    }
    else{
      gSpx_ShootMaster.set(ControlMode.PercentOutput, 0);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
