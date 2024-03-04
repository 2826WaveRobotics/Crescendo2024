// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.transport.Transport;

/**
 * A command used for launching when the robot is aligned to the speaker and directly in front of the start line.
 */
public class LaunchStartCommand extends ParallelCommandGroup {
    public LaunchStartCommand(Launcher launchSubsystem, Transport transportSubsystem) {
        // addRequirements(launchSubsystem, transportSubsystem);
        addCommands(
            new WaitCommand(1.5),
            new SequentialCommandGroup(
                new SetLauncherAngle(launchSubsystem, 50),
                new WaitCommand(0.5),
                new LaunchNote(launchSubsystem, transportSubsystem)
            )
        );
    }
}
