// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.transport.LaunchNote;
import frc.robot.commands.transport.SetLauncherAngle;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.transport.Transport;

/**
 * A command used for launching when the robot is aligned to the speaker and directly in front of it.  
 * This command runs for 1.0 seconds.
 */
public class LaunchCloseCommand extends SequentialCommandGroup {
    public LaunchCloseCommand() {
        // addRequirements(Launcher.getInstance(), Transport.getInstance());
        addCommands(
            new SetLauncherAngle(60),
            new LaunchNote()
        );
    }
}
