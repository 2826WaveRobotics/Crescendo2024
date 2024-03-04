// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.transport.Transport;

/**
 * A command that runs the launcher and upper launch roller for half a second.
 */
public class LaunchNote extends Command {
  private Launcher launchSubsystem;
  private Transport transportSubsystem;
  boolean m_launching;
  private double m_startTime;

  /** Creates a new LaunchNote. */
  public LaunchNote(Launcher launcherSubsystem, Transport transportSubsystem) {
    this.launchSubsystem = launcherSubsystem;
    this.transportSubsystem = transportSubsystem;
    // addRequirements(launcherSubsystem, transportSubsystem); // Temporary
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transportSubsystem.setUpperTransportSpeed(20.);

    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transportSubsystem.setUpperTransportSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = Timer.getFPGATimestamp();
    return ((currentTime - m_startTime) > 1.25);
  }
}
