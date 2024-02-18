// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTimed;
import frc.robot.commands.LaunchNote;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchAndDrive extends SequentialCommandGroup {
    private Launcher m_launcher;
    private Swerve m_swerve;

  public LaunchAndDrive(Swerve swerveSubsystem, Launcher launcherSubsystem, double driveTime) {
    m_swerve = swerveSubsystem;
    m_launcher = launcherSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveTimed(m_swerve, driveTime), new LaunchNote(m_launcher));
  }
}
