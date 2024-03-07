// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;

/**
 * A command that sets the launcher angle and finishes when it's at the target angle.  
 * TODO: Currently, this command runs for 0.25 seconds and finishes.
 * This could eventually be replaced with finishing when the launcher is at the target angle, but this is far more consistent for now.
 */
public class SetLauncherAngle extends Command {
  private double angle;
  private double m_startTime;

  /**
   * Creates a new SetLauncherAngle.
   * @param angle The launcher angle in degrees
   */
  public SetLauncherAngle(double angle) {
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Launcher.getInstance().setLauncherAngle(Rotation2d.fromDegrees(angle));
    m_startTime = Timer.getFPGATimestamp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = Timer.getFPGATimestamp();
    return ((currentTime - m_startTime) > 0.25);
  }
}
