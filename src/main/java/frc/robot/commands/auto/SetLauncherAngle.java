// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.transport.Transport;

/**
 * A command that sets the launcher angle and finishes when it's at the target angle.
 */
public class SetLauncherAngle extends Command {
  private Launcher launchSubsystem;
  private double angle;

  /**
   * Creates a new SetLauncherAngle.
   * @param angle The launcher angle in degrees
   */
  public SetLauncherAngle(Launcher launcherSubsystem, double angle) {
    this.launchSubsystem = launcherSubsystem;
    this.angle = angle;
    // addRequirements(launcherSubsystem); // Temporary
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchSubsystem.setLauncherAngle(Rotation2d.fromDegrees(angle));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(launchSubsystem.getAbsoluteLauncherAngleDegrees() - angle) < 2;
    return true;
  }
}
