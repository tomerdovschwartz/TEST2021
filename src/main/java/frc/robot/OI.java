// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;

/** Add your docs here. */
public class OI {

    Joystick driverController = new Joystick(RobotMap.DRIVER_CONTROLLER);
    Button button = new JoystickButton(driverController,6);
    
    public double GetDriverRawAxis(int axis)
    {
        return driverController.getRawAxis(axis);
    }

    public void GetButton()
    {
        if(driverController.getRawButtonPressed(1)){
            button.whenPressed(new ExampleCommand(true));
        } else if(driverController.getRawButtonReleased(1)){
            button.whenPressed(new ExampleCommand(false));
        }
    }

    public Joystick getJoystick()
    {
        return driverController;
    }
}
