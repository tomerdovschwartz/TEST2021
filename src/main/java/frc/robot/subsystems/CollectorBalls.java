// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;


/** Add your docs here. */
public class CollectorBalls extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX gSpx_CollectMaster = null;
  int velocity=1;

  public CollectorBalls()
  {
   gSpx_CollectMaster = new VictorSPX(RobotMap.COLLECT_MASTER);
  }

  public void startCollect(){
    velocity=1;
  if (new OI().getJoystick().getRawButton(7))
  velocity=-1;

      SmartDashboard.putBoolean("Collect Work", !(RobotMap.CollectWorkKey));
      gSpx_CollectMaster.set(ControlMode.PercentOutput, 0.45*velocity);
    

}

public void stopCollect(){
  SmartDashboard.putBoolean("Collect Work", RobotMap.CollectWorkKey);
  gSpx_CollectMaster.set(ControlMode.PercentOutput, 0);
}
}
