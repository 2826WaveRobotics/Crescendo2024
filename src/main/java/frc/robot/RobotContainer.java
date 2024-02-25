// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//other example repos: 4607, 3457
//this code is based on team frc3512 SwerveBot-2022

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.NoteManagement;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopLauncher;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.NoteManagement.NoteState;
import frc.robot.commands.auto.LaunchCloseCommand;
import frc.robot.commands.auto.LaunchStartCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Subsystems */
  private final Swerve swerveSubsystem = new Swerve();
  private final Launcher launcherSubsystem = new Launcher();
  private final Transport transportSubsystem = new Transport();
  private final Elevator elevatorSubsystem = new Elevator();

  public final Lighting lighting = new Lighting();

  private final NoteManagement noteManagementCommand = new NoteManagement(
    elevatorSubsystem,
    transportSubsystem,
    new Trigger(() -> false)
  );

  public NoteState getNoteState() {
    return noteManagementCommand.getNoteState();
  }

  /**
   * Gets the current robot speed in meters per second.
   * @return
   */
  public double getRobotSpeed() {
    return swerveSubsystem.getRobotSpeed();
  }

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
      new TeleopSwerve(
        swerveSubsystem,
        () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value),
        () -> -driver.getRawAxis(XboxController.Axis.kRightX.value),
        () -> !driver.leftBumper().getAsBoolean()
      ));
    
    transportSubsystem.setDefaultCommand(
      new TeleopIntake(
        transportSubsystem,
        () -> operator.getRightTriggerAxis() > Constants.triggerDeadband,
        () -> operator.getLeftTriggerAxis() > Constants.triggerDeadband,
        () -> operator.b().getAsBoolean()
      ));

    elevatorSubsystem.setDefaultCommand(noteManagementCommand);

    launcherSubsystem.setDefaultCommand(
      new TeleopLauncher(
        launcherSubsystem,
        operator.y(),
        operator.a(),
        operator.b(),
        operator.povUp(),
        operator.povDown(),
        operator.povRight(),
        operator.povLeft()
      )
    );
    
    // Configure the button bindings
    configureButtonBindings();
    NamedCommands.registerCommand("Launch close", new LaunchCloseCommand(launcherSubsystem, transportSubsystem));
    NamedCommands.registerCommand("Launch start line", new LaunchStartCommand(launcherSubsystem, transportSubsystem));
    NamedCommands.registerCommand("Enable intake", new InstantCommand(() -> transportSubsystem.setActive(true)));
    NamedCommands.registerCommand("Slow constant launch", new InstantCommand(() -> {
      launcherSubsystem.launchRollersSlow();
      transportSubsystem.setUpperTransportSpeed(10.0);
    }));
    NamedCommands.registerCommand("Launch rollers fast", new InstantCommand(launcherSubsystem::launchRollersFast));
    NamedCommands.registerCommand("Launch rollers slow", new InstantCommand(launcherSubsystem::launchRollersSlow));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    driver.y().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
    driver.b().onTrue(new InstantCommand(swerveSubsystem::updateOdometryPose));

    /* Operator Buttons */
    operator.a().onTrue(new InstantCommand(launcherSubsystem::launchRollersFast));
    operator.leftBumper().onTrue(new InstantCommand(launcherSubsystem::launchRollersSlow));
    operator.rightBumper().onTrue(new InstantCommand(launcherSubsystem::launchRollersFast));  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
