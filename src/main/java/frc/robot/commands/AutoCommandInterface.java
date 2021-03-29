// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

// This interface will allow autonomous commands to provide the associated initial Pose2d

public interface AutoCommandInterface extends Command {
    public Pose2d getInitialPose();
/*
    public default void plotTrajectory(TrajectoryPlotter plotter) {
        plotter.clear();
*/
    }
