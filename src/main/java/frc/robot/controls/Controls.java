package frc.robot.controls;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.climber.ClimberControls;
import frc.robot.commands.control.PathfindToAmpAndLaunch;
import frc.robot.commands.control.PathfindToSpeakerAndLaunch;
import frc.robot.commands.transport.SetLauncherAngle;
import frc.robot.commands.transport.SetLauncherSpeed;
import frc.robot.controls.SwerveAlignmentController.AlignmentMode;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportState;

/**
 * This class manages the controls of the robot. We use it so all the button bindings can be in one place.
 */
public class Controls {
    private static Controls instance = null;
    public static Controls getInstance() {
        if (instance == null) {
            instance = new Controls();
        }
        return instance;
    }
    
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private Controls() {
        // This is a singleton class.
    }

    /** Sets the rumble on the driver controller. */
    public void setDriverRumble(double left, double right) {
        driver.getHID().setRumble(RumbleType.kLeftRumble, left);
        driver.getHID().setRumble(RumbleType.kRightRumble, right);
    }

    /** Sets the rumble on the operator controller. */
    public void setOperatorRumble(double left, double right) {
        operator.getHID().setRumble(RumbleType.kLeftRumble, left);
        operator.getHID().setRumble(RumbleType.kRightRumble, right);
    }

    /**
     * Configures the controls for the robot. This is where we bind commands to buttons and add joystick actions.
     */
    public void configureControls() {
        Swerve swerveSubsystem = Swerve.getInstance();
        Launcher launcherSubsystem = Launcher.getInstance();
        Transport transportSubsystem = Transport.getInstance();
        
        DriverStation.silenceJoystickConnectionWarning(true);

        /*//////////////////////////*/
        /*     Driver Controls      */
        /*//////////////////////////*/
        BooleanSupplier fieldRelative = driver.leftBumper();
        swerveSubsystem.setDefaultCommand(
            DriveCommands.joystickDrive(
                swerveSubsystem,
                () ->  driver.getLeftX(),
                () -> -driver.getLeftY(),
                driver.leftTrigger(0.2),
                () -> -driver.getRightX(),
                () -> !fieldRelative.getAsBoolean()
            )
        );

        driver.x().onTrue(new InstantCommand(swerveSubsystem::stopWithX, swerveSubsystem));
        driver.start().onTrue(new InstantCommand(swerveSubsystem::resetRotation, swerveSubsystem).ignoringDisable(true));

        // Snap alignment
        SwerveAlignmentController alignmentController = SwerveAlignmentController.getInstance();
        driver.povUp().onTrue(new InstantCommand(() -> alignmentController.setAlignmentMode(AlignmentMode.Forward)));
        driver.povDown().onTrue(new InstantCommand(() -> alignmentController.setAlignmentMode(AlignmentMode.Backward)));
        driver.povLeft().onTrue(new InstantCommand(() -> alignmentController.setAlignmentMode(AlignmentMode.Left)));
        driver.povRight().onTrue(new InstantCommand(() -> alignmentController.setAlignmentMode(AlignmentMode.Right)));
        
        Trigger autoSpeakerAim = driver.rightBumper();
        autoSpeakerAim.onTrue(new InstantCommand(() -> alignmentController.setAlignmentMode(AlignmentMode.AllianceSpeaker)));

        // We use DeferredCommands because for some reason it partially fixes performance issues caused
        // by these pathfinding commands. It's not an ideal solution, but it works for now.
        driver.y().whileTrue(new DeferredCommand(PathfindToSpeakerAndLaunch::new, Set.of()));
        driver.b().whileTrue(new DeferredCommand(PathfindToAmpAndLaunch::new, Set.of()));
        
        driver.a().onTrue(new InstantCommand(() -> {
            if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                swerveSubsystem.setPose(new Pose2d(
                    new Translation2d(1.37, 5.55),
                    new Rotation2d()
                ));
            } else {
                swerveSubsystem.setPose(new Pose2d(
                    new Translation2d(Constants.fieldLengthMeters - 1.37, 5.55),
                    Rotation2d.fromDegrees(180)
                ));
            }
        }));

        /*//////////////////////////*/
        /*    Operator Controls     */
        /*//////////////////////////*/
        
        transportSubsystem.setDefaultCommand(new TeleopIntake(() -> -operator.getLeftY()));
    
        operator.b().onTrue(new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.IntakingNote)));
        operator.a().onTrue(new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.Stopped)));

        AutomaticLauncherControl launcherControl = AutomaticLauncherControl.getInstance();
        autoSpeakerAim.whileTrue(new RepeatCommand(new InstantCommand(launcherControl::autoAlign)));

        BooleanSupplier testMode = operator.x();

        // Test angle mode
        launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(25));
        operator.povUp().whileTrue(new RepeatCommand(new InstantCommand(() -> {
            if(!testMode.getAsBoolean()) return; // Test angle mode
            double launcherAngle = launcherSubsystem.launcherAngle + 0.15;
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(launcherAngle));
        }))).onTrue(new InstantCommand(() -> {
            // Podium speaker preset
            if(testMode.getAsBoolean()) return; // Test angle mode
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(42.1));
            launcherSubsystem.setLauncherSpeed(2880, true);
        }));

        operator.povDown().whileTrue(new RepeatCommand(new InstantCommand(() -> {
            if(!testMode.getAsBoolean()) return; // Test angle mode
            double launcherAngle = launcherSubsystem.launcherAngle - 0.15;
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(launcherAngle));
        }))).onTrue(new InstantCommand(() -> {
            // Slow preset
            if(testMode.getAsBoolean()) return; // Test angle mode
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(21));
            launcherSubsystem.launchRollersSlow();
        }));

        operator.povRight().whileTrue(new RepeatCommand(new InstantCommand(() -> {
            if(!testMode.getAsBoolean()) return; // Test angle mode
            launcherSubsystem.setLauncherSpeed(launcherSubsystem.topRollerSpeed + 20, true);
        }))).onTrue(new InstantCommand(() -> {
            // Close speaker preset
            if(testMode.getAsBoolean()) return; // Test angle mode
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(59.95));
            launcherSubsystem.setLauncherSpeed(4500, true);
        }));

        operator.povLeft().whileTrue(new RepeatCommand(new InstantCommand(() -> {
            if(!testMode.getAsBoolean()) return; // Test angle mode
            launcherSubsystem.setLauncherSpeed(launcherSubsystem.topRollerSpeed - 20, true);
        }))).onTrue(new InstantCommand(() -> {
            // Amp preset
            if(testMode.getAsBoolean()) return; // Test angle mode
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(58.95));
            launcherSubsystem.setLauncherSpeed(1540, false);
        }));

        operator.rightTrigger(0.2).whileTrue(new RepeatCommand(new InstantCommand(
            () -> Transport.getInstance().attemptTransitionToState(TransportState.LaunchingNote)
        ))).onFalse(new InstantCommand(
            () -> Transport.getInstance().attemptTransitionToState(TransportState.Stopped)
        ));
        
        // Lob shot
        operator.leftTrigger(0.2).onTrue(new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetLauncherAngle(60.0),
                new SetLauncherSpeed(5000, false)
            ),
            new InstantCommand(
                () -> Transport.getInstance().attemptTransitionToState(TransportState.LaunchingNote)
            )
        )).onFalse(new InstantCommand(
            () -> Transport.getInstance().attemptTransitionToState(TransportState.Stopped)
        ));
        
        operator.start().whileTrue(Commands.startEnd(
            () -> launcherSubsystem.setLauncherSpeed(-6800, false),
            () -> launcherSubsystem.setLauncherSpeed(0, false)
        ));

        operator.y().onTrue(new InstantCommand(() -> Superstructure.getInstance().climbersUp()));
        Climber.getInstance().setDefaultCommand(new ClimberControls(
            operator.leftBumper(),
            operator.rightBumper(),
            operator.x()
        ));
    }
}
