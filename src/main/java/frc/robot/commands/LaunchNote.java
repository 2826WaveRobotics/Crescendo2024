// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Launcher;

public class LaunchNote extends Command {
  private Launcher m_launcher;
  boolean m_launching;

  /** Creates a new LaunchNote. */
  public LaunchNote(Launcher launcher) {
    this.m_launcher = launcher;
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launching = true;
    this.m_launcher.setLaunchNoteOut();
    this.m_launcher.launchRollersSlow();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_launching) {
      m_launcher.launchRollersFast();      
    }
    this.m_launcher.runLauncher();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_launcher.launchRollersSlow();
    return false;
  }
}
