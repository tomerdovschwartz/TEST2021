// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class OI {

    Joystick joystick_controller = new Joystick(RobotMap.DRIVER_CONTROLLER);
    Button button1 = new JoystickButton(joystick_controller, 1);
    Button button2 = new JoystickButton  (joystick_controller, 2 );
    Button button3 = new JoystickButton(joystick_controller, 3);
    POVButton povbutton1= new POVButton(joystick_controller,180);
    POVButton povbutton2= new POVButton(joystick_controller,0);

    public double GetDriverRawAxis(int axis) {
        return joystick_controller.getRawAxis(axis);
    }

  
    public Joystick getJoystick() {
        return joystick_controller;
    }
}
