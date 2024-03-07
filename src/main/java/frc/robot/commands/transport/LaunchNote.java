// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.NoteState;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportState;

/**
 * A command that runs the launcher and upper launch roller for 0.75 seconds.
 */
public class LaunchNote extends Command {
  private double m_startTime;

  /** Creates a new LaunchNote. */
  public LaunchNote() {
    if(Superstructure.getInstance().getNoteState() != NoteState.ReadyToLaunch) cancel();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Transport.getInstance().attemptTransitionToState(TransportState.LaunchingNote);

    m_startTime = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Transport.getInstance().attemptTransitionToState(TransportState.Stopped);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = Timer.getFPGATimestamp();
    return ((currentTime - m_startTime) > 0.75);
  }
}
