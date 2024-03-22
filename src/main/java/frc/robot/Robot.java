// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.Controls;
import frc.robot.controls.VibrationFeedback;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  /**
   * The command instance for the robot's autonomous command state.
   */
  private Command autonomousCommand;
  /**
   * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm,
   * very little robot logic should actually be handled in the Robot periodic methods (other than the scheduler calls).
   * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
   */
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "Crescendo2024"); // Set a metadata value

    // NOTE: If there's an error with "BuildConstants cannot be resolved", build the program.
    // The BuildConstants file is generated on build and not checked into version control since it can be invalid, cause merge conflicts, etc.
    Logger.recordMetadata("GitInformation", 
      "Hash " +
      BuildConstants.GIT_SHA +
      " on " + 
      BuildConstants.GIT_BRANCH +
      (BuildConstants.DIRTY == 1 ? "*" : "") // Add a "*" if the build is dirty. TODO: Fix warnings here since DIRTY is recognized as readonly even though it shouldn't be.
    );
    Logger.recordMetadata("BuiltOn", BuildConstants.BUILD_DATE); // Set a metadata value

    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        // setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    // Instantiate our RobotContainer.  This will perform all our button bindings, put our
    // autonomous chooser on the dashboard, and do anything else required for initialization.
    robotContainer = new RobotContainer();
    
    // Forward the Limelight camera ports
    // More information: https://docs.limelightvision.io/docs/docs-limelight/getting-started/best-practices#event-preparation-checklist
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    robotContainer.updateAutoPublisher();
    
    setNetworkTablesFlushEnabled(!DriverStation.isFMSAttached());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      Superstructure.getInstance().resetSubsystemsForAuto();
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Climber.getInstance().setLeftSpeed(MathUtil.applyDeadband(Controls.getInstance().operator.getLeftX(), 0.15) * 5600);
    Climber.getInstance().setRightSpeed(MathUtil.applyDeadband(Controls.getInstance().operator.getRightX(), 0.15) * 5600);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    Superstructure.getInstance().resetSubsystemsForTeleop();
    VibrationFeedback.getInstance().teleopInit();
  }

  @Override
  public void teleopExit() {
    VibrationFeedback.getInstance().teleopExit();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
  }
}
