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

    Joystick joystick_controller = new Joystick(RobotMap.DRIVER_CONTROLLER);
    Button button = new JoystickButton(joystick_controller, 6);

    public double GetDriverRawAxis(int axis) {
        return joystick_controller.getRawAxis(axis);
    }

    public void GetButton() {
        // joystick_controller.getRawButtonPressed(btnNumber) - get true or false if the
        // button
        // that is specificed is pressed
        // joysticl_controller.getRawButtonReleased(btnNumber) - get true if the button
        // is begin released.
        if (joystick_controller.getRawButtonPressed(1)) {
            button.whenPressed(new ExampleCommand(true));
        } else if (joystick_controller.getRawButtonReleased(1)) {
            button.whenPressed(new ExampleCommand(false));
        }
    }

    public Joystick getJoystick() {
        return joystick_controller;
    }
}
