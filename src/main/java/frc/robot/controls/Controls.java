package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.control.PathfindToAmpAndLaunch;
import frc.robot.commands.control.PathfindToSpeakerAndLaunch;
import frc.robot.controls.SwerveAlignmentController.AlignmentMode;
import frc.robot.subsystems.Superstructure;
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

    /**
     * Configures the controls for the robot. This is where we bind commands to buttons and add joystick actions.
     */
    public void configureControls() {
        Swerve swerveSubsystem = Swerve.getInstance();
        Launcher launcherSubsystem = Launcher.getInstance();
        Transport transportSubsystem = Transport.getInstance();

        /*//////////////////////////*/
        /*     Driver Controls      */
        /*//////////////////////////*/
        swerveSubsystem.setDefaultCommand(
            DriveCommands.joystickDrive(
                swerveSubsystem,
                () ->  driver.getLeftX(),
                () -> -driver.getLeftY(),
                () -> -driver.getRightX(),
                () -> !driver.leftBumper().getAsBoolean()
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
        
        driver.rightBumper().onTrue(new InstantCommand(() -> alignmentController.setAlignmentMode(AlignmentMode.AllianceSpeaker)));

        driver.y().whileTrue(new PathfindToSpeakerAndLaunch());
        driver.b().whileTrue(new PathfindToAmpAndLaunch());

        /*//////////////////////////*/
        /*    Operator Controls     */
        /*//////////////////////////*/

        Superstructure superstructure = Superstructure.getInstance();
        operator.leftBumper().onTrue(new InstantCommand(superstructure::setupClimb));
        operator.leftBumper().onFalse(new InstantCommand(superstructure::climb));
        operator.rightBumper().onTrue(new InstantCommand(superstructure::unclimbStart));
        operator.rightBumper().onFalse(new InstantCommand(superstructure::unclimbEnd));
        
        transportSubsystem.setDefaultCommand(new TeleopIntake(() -> -operator.getLeftY()));
    
        operator.b().onTrue(new InstantCommand(() -> {
            if(transportSubsystem.getCurrentState() == TransportState.Stopped) {
                transportSubsystem.attemptTransitionToState(TransportState.IntakingNote);
            } else {
                transportSubsystem.attemptTransitionToState(TransportState.Stopped);
            }
        }));

        final boolean TEST_LAUNCHER = true;
        if(TEST_LAUNCHER) {
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(45));
            operator.povUp().whileTrue(new RepeatCommand(new InstantCommand(() -> {
                double launcherAngle = launcherSubsystem.launcherAngle + 0.15;
                launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(launcherAngle));
            })));
            operator.povDown().whileTrue(new RepeatCommand(new InstantCommand(() -> {
                double launcherAngle = launcherSubsystem.launcherAngle - 0.15;
                launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(launcherAngle));
            })));

            operator.povRight().whileTrue(new RepeatCommand(new InstantCommand(() ->
                launcherSubsystem.setLauncherSpeed(launcherSubsystem.launcherSpeed + 20)
            )));
            operator.povLeft().whileTrue(new RepeatCommand(new InstantCommand(() ->
                launcherSubsystem.setLauncherSpeed(launcherSubsystem.launcherSpeed - 20)
            )));
        } else {
            operator.povUp().onTrue(new InstantCommand(() -> {
                launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(35));
                launcherSubsystem.setLauncherSpeed(4500);
            }));
            operator.povDown().onTrue(new InstantCommand(() -> {
                launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(50));
                launcherSubsystem.launchRollersSlow();
            }));
        }

        operator.y().onTrue(new InstantCommand(superstructure::launchNote));
    }
}
