// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//other example repos: 4607, 3457
//this code is based on team frc3512 SwerveBot-2022

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportState;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.commands.auto.LaunchCloseCommand;
import frc.robot.commands.auto.LaunchStartCommand;
import frc.robot.commands.transport.LaunchNote;
import frc.robot.commands.transport.SetLauncherAngle;
import frc.robot.controls.Controls;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    Limelight.getInstance();
    Swerve swerveSubsystem = Swerve.getInstance();
    
    registerAutoCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption("Drive SysId (Quasistatic Forward)", swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)", swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)",     swerveSubsystem.sysIdDynamic    (SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)",     swerveSubsystem.sysIdDynamic    (SysIdRoutine.Direction.kReverse));
    
    autoChooser.addOption("Set odometry: Front of blue speaker", new InstantCommand(() ->
      swerveSubsystem.setPose(new Pose2d(
        new Translation2d(1.37, 5.55),
        new Rotation2d()
      )
    )));
    autoChooser.addOption("Set odometry: Front of red speaker", new InstantCommand(() ->
      swerveSubsystem.setPose(new Pose2d(
        new Translation2d(Constants.fieldLengthMeters - 1.37, 5.55),
        Rotation2d.fromDegrees(180)
      )
    )));

    Controls.getInstance().configureControls();
    
    LiveWindow.disableAllTelemetry();
  }

  /**
   * Registers the NamedCommands used for PathPlanner auto commands.
   */
  private void registerAutoCommands() {
    Launcher launcherSubsystem = Launcher.getInstance();
    Transport transportSubsystem = Transport.getInstance();

    // All autos
    NamedCommands.registerCommand("Launch close", new LaunchCloseCommand());
    NamedCommands.registerCommand("Launch start line", new LaunchStartCommand());
    NamedCommands.registerCommand("Enable intake", new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote)));

    // 3 note
    NamedCommands.registerCommand("Launch 3 note", new SequentialCommandGroup(
      new ParallelCommandGroup(
        // TODO: Find real values
        new SetLauncherAngle(50),
        new InstantCommand(() -> launcherSubsystem.setLauncherSpeed(3000, true))
      ),
      new LaunchNote()
    ));

    // Sweep auto
    NamedCommands.registerCommand("Sweep transport", new InstantCommand(() -> {
      launcherSubsystem.setLauncherSpeed(1200, true);
      transportSubsystem.attemptTransitionToState(TransportState.SweepTransport);
    }));
    NamedCommands.registerCommand("Sweep launch", new SequentialCommandGroup(
      new ParallelCommandGroup(
        // TODO: Find real values
        new SetLauncherAngle(28),
        new InstantCommand(() -> launcherSubsystem.setLauncherSpeed(3300, true))
      ),
      new LaunchNote()
    ));

    // Center notes auto
    NamedCommands.registerCommand("Center notes launch", new SequentialCommandGroup(
      new ParallelCommandGroup(
        // TODO: Find real values
        new SetLauncherAngle(25),
        new InstantCommand(() -> launcherSubsystem.setLauncherSpeed(4100, true))
      ),
      new LaunchNote()
    ));
    
    // Mostly for testing
    NamedCommands.registerCommand("Launch rollers fast", new InstantCommand(launcherSubsystem::launchRollersFast));
    NamedCommands.registerCommand("Launch rollers slow", new InstantCommand(launcherSubsystem::launchRollersSlow));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}