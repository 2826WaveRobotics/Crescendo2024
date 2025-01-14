// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//other example repos: 4607, 3457
//this code is based on team frc3512 SwerveBot-2022

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;
import frc.robot.subsystems.lighting.Lighting;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportState;
import frc.robot.subsystems.vision.Limelight;
import frc.lib.util.ShuffleboardContent;
import frc.robot.commands.transport.LaunchNote;
import frc.robot.commands.transport.SetLauncherState;
import frc.robot.controls.AutomaticLauncherControl;
import frc.robot.controls.Controls;
import frc.robot.controls.SwerveAlignmentController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final LoggedDashboardChooser<Command> autoChooser;

  private StringPublisher autoDataPublisher = null;

  /** The swerve subsystem. Only used over Swerver.getInstance() to let AdvantageKit know it should scan this class for AutoLog methods. */
  private Swerve swerveSubsystem = Swerve.getInstance();
  /** The launcher subsystem. Only used over Launcher.getInstance() to let AdvantageKit know it should scan this class for AutoLog methods. */
  private Launcher launcherSubsystem = Launcher.getInstance();

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    Limelight.getInstance();
    
    registerAutoCommands();
    publishAutoData();

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

    autoChooser.getSendableChooser().onChange(swerveSubsystem::selectedAutoChanged);

    ShuffleboardContent.competitionTab.add("Auto to run", autoChooser.getSendableChooser())
      .withPosition(1, 4)
      .withSize(3, 1);

    Controls.getInstance().configureControls();

    // Initialize the lighting by calling getInstance() because nothing else in the code does.
    Lighting.getInstance();
    
    LiveWindow.disableAllTelemetry();
  }

  private Command autonomousLauncherConfigCommand = null;

  /**
   * Registers the NamedCommands used for PathPlanner auto commands.
   */
  private void registerAutoCommands() {
    Transport transportSubsystem = Transport.getInstance();

    // All autos
    LauncherState closeState = new LauncherState(3560, 59.2, true);
    NamedCommands.registerCommand("Launch close prep", new ScheduleCommand(new SetLauncherState(closeState)));
    NamedCommands.registerCommand("Launch close", new SequentialCommandGroup(
      new SetLauncherState(closeState),
      new LaunchNote()
    ));
    NamedCommands.registerCommand("Launch", new SequentialCommandGroup(
      new WaitUntilCommand(() -> transportSubsystem.getCurrentState() != TransportState.MovingNote),
      new ScheduleCommand(new LaunchNote())
    ));
    NamedCommands.registerCommand("Intake/Launch", new SequentialCommandGroup(
      new WaitUntilCommand(() -> transportSubsystem.getCurrentState() != TransportState.MovingNote),
      new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote))
    ));
    NamedCommands.registerCommand("Run transport delayed", new ScheduleCommand(new SequentialCommandGroup(
      new WaitCommand(0.5),
      new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote))
    )));

    // ------------------------------------------------------------------
    // -------------------------- Center sweep --------------------------
    // ------------------------------------------------------------------
    NamedCommands.registerCommand("Sweep transport", new InstantCommand(() -> {
      launcherSubsystem.setLauncherSpeed(1200, true);
      transportSubsystem.attemptTransitionToState(TransportState.SweepTransport);
    }));
    LauncherState sweepState = new LauncherState(4800, 45, true);
    NamedCommands.registerCommand("Prep sweep launch", new SequentialCommandGroup(
      new SetLauncherState(sweepState),
      new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote))
    ));

    // ------------------------------------------------------------------
    // ----------------------------- 5 note -----------------------------
    // ------------------------------------------------------------------
    NamedCommands.registerCommand("Prep 5 note", new SequentialCommandGroup(
      new SetLauncherState(Constants.Controls.SpeakerCloseState),
      // Launch and intake
      new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote))
    ));

    // ------------------------------------------------------------------
    // ----------------------------- 7 note -----------------------------
    // ------------------------------------------------------------------
    NamedCommands.registerCommand("7 note 1", new SequentialCommandGroup(
      // Launch first
      // ------- FIRST NOTE PARAMETERS -------
      new SetLauncherState(new LauncherState(3900, 53.2, true)),
      new LaunchNote(),

      // Prep second
      new ScheduleCommand(new ParallelCommandGroup(
        new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote)),
        // ------- SECOND NOTE PARAMETERS -------
        new SetLauncherState(new LauncherState(4100, 50.0, true))
      ))
    ));
    NamedCommands.registerCommand("7 note 2", new ScheduleCommand(new SequentialCommandGroup(
      // Slight delay
      new WaitCommand(0.15),

      // Launch second
      new LaunchNote(),

      // Prep third
      new ParallelCommandGroup(
        new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote)),
        // ------- THIRD NOTE PARAMETERS -------
        new SetLauncherState(new LauncherState(4100, 50.0, true))
      )
    )));
    NamedCommands.registerCommand("7 note 3-4", new ScheduleCommand(new SequentialCommandGroup(
      // Slight delay
      new WaitCommand(0.15),

      // Launch third
      new LaunchNote(),

      // Prep fourth
      new ParallelCommandGroup(
        new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote)),
        // ------- FOURTH NOTE PARAMETERS -------
        new SetLauncherState(new LauncherState(5400, 42.0, true))
      ),

      // Wait for fourth
      new WaitCommand(0.3),

      // Launch fourth
      new LaunchNote(),

      // Prep fifth
      new ParallelCommandGroup(
        new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote)),
        // ------- FIFTH NOTE PARAMETERS -------
        new SetLauncherState(new LauncherState(5500, 22.0, true))
      )
    )));
    NamedCommands.registerCommand("7 note 5", new ScheduleCommand(new ParallelCommandGroup(
      // No delay since note is already ready

      // Launch fifth
      new LaunchNote(),

      // Prep sixth
      new ParallelCommandGroup(
        new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote)),
        // ------- SIXTH NOTE PARAMETERS -------
        new SetLauncherState(new LauncherState(5500, 22.0, true))
      )
    )));
    NamedCommands.registerCommand("7 note 6", new ScheduleCommand(new SequentialCommandGroup(
      // No delay since note is already ready

      // Launch sixth
      new LaunchNote(),

      // Prep seventh
      new ParallelCommandGroup(
        new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote)),
        // ------- SEVENTH NOTE PARAMETERS -------
        new SetLauncherState(new LauncherState(5500, 22.0, true))
      )
    )));
    NamedCommands.registerCommand("7 note 7", new ScheduleCommand(new ParallelCommandGroup(
      // No delay since note is already ready

      // Launch seventh
      new LaunchNote()
    )));


    NamedCommands.registerCommand("Start launcher automation", new InstantCommand(() -> {
      if(autonomousLauncherConfigCommand != null) autonomousLauncherConfigCommand.cancel();
      autonomousLauncherConfigCommand = new RepeatCommand(new InstantCommand(() -> {
        SwerveAlignmentController.getInstance().updateDistanceAndGetAngle();
        AutomaticLauncherControl.getInstance().autoAlign(true);
      }));
      autonomousLauncherConfigCommand.schedule();
    }));
    NamedCommands.registerCommand("Stop launcher automation", new InstantCommand(() -> {
      if(autonomousLauncherConfigCommand != null) autonomousLauncherConfigCommand.cancel();
    }));
  }

  public void teleopEnable() {
    if(autonomousLauncherConfigCommand != null) autonomousLauncherConfigCommand.cancel();
  }

  public void updateAutoPublisher() {
    if(autoDataPublisher != null && (DriverStation.isFMSAttached() || DriverStation.isEnabled())) {
      autoDataPublisher.close();
      autoDataPublisher = null;
    }
  }

  /**
   * Publishes the autonomous data to NetworkTables so our auto dashboard can display them.
   */
  private void publishAutoData() {
    if(DriverStation.isFMSAttached()) return; // Do not send auto data when connected to FMS

    StringBuilder jsonData = new StringBuilder();
    jsonData.append("{\"autoChoices\": [");
    for (String choice : AutoBuilder.getAllAutoNames()) {
      jsonData.append("{\"name\": \"").append(choice).append("\", \"poses\": [");
      List<Pose2d> poses = new ArrayList<>();

      for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(choice)) {
        path.preventFlipping = true;
        poses.addAll(path.getPathPoses());
      }
      
      try {
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(choice);
        if(poses.isEmpty()) poses.add(startingPose);
        else poses.set(0, startingPose);
      } catch(RuntimeException e) {
        // Do nothing
      }
      
      for (Pose2d pose : poses) {
        jsonData
          .append("{\"x\": ")
          .append(Math.round(pose.getTranslation().getX() * 1000) / 1000.0)
          .append(", \"y\": ")
          .append(Math.round(pose.getTranslation().getY() * 1000) / 1000.0)
          .append(", \"rot\": ")
          .append(Math.round(pose.getRotation().getRadians() * 1000) / 1000.0)
          .append("},");
      }
      if(!poses.isEmpty()) jsonData.deleteCharAt(jsonData.length() - 1);
      jsonData.append("]},");
    }
    jsonData.deleteCharAt(jsonData.length() - 1);
    jsonData.append("]}");
    autoDataPublisher = NetworkTableInstance.getDefault()
      .getTable("2826AutoDashboard")
      .getStringTopic("AutoData")
      .publish();

    autoDataPublisher.accept(jsonData.toString());
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