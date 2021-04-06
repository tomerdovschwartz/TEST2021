// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class ChangeAngleDirection extends SubsystemBase {
  /** Creates a new ChangeAngleDirection. */
  private VictorSPX gSpx_ChangeDirectionMaster = null;


  public ChangeAngleDirection() {
    gSpx_ChangeDirectionMaster = new VictorSPX(RobotMap.CHANGEDIRECTION_MASTER);

  }
  
  public void startChangeDirection(){
    
    if (new OI().getJoystick().getPOV()==180){
      gSpx_ChangeDirectionMaster.set(ControlMode.PercentOutput, 0.15);
    }
    else{
    if (new OI().getJoystick().getPOV()==0){
      gSpx_ChangeDirectionMaster.set(ControlMode.PercentOutput, -0.15);
 
    }
    else{
      gSpx_ChangeDirectionMaster.set(ControlMode.PercentOutput, 0);
  }
}
  }

 
  public void stopChangeAngle() {
    gSpx_ChangeDirectionMaster.set(ControlMode.PercentOutput, 0);
  }
}
