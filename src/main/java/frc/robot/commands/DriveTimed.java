// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveTimed extends Command {
      private Swerve m_swerveSubsystem;
      private double m_duration;
      private double m_startTime;

  /** Creates a new DriveTimed. */
  public DriveTimed(Swerve swerveSubsystem, double duration) {
    m_swerveSubsystem = swerveSubsystem;
    m_duration = duration;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values */
    double xVelocity = 0.5;
    double yVelocity = 0.0;
    double rotationVal = 0.0;

    /* Drive */
    m_swerveSubsystem.drive( 
      new Translation2d(xVelocity, yVelocity).times(Constants.Swerve.maxSpeed),
      rotationVal * Constants.Swerve.maxAngularVelocity, false,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Stop Drive */
    m_swerveSubsystem.drive( 
      new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed), 
      0.0,false,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = Timer.getFPGATimestamp();
    return ((currentTime - m_startTime) > m_duration);
  }
}
