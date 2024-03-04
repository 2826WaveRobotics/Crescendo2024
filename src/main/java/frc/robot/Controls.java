package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopIntake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.transport.Transport;

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
        driver.y().onTrue(new InstantCommand(swerveSubsystem::resetRotation, swerveSubsystem).ignoringDisable(true));

        /*//////////////////////////*/
        /*    Operator Controls     */
        /*//////////////////////////*/

        Superstructure superstructure = Superstructure.getInstance();
        operator.leftBumper().onTrue(new InstantCommand(superstructure::setupClimb));
        operator.leftBumper().onFalse(new InstantCommand(superstructure::climb));
        operator.rightBumper().onTrue(new InstantCommand(superstructure::unclimbStart));
        operator.rightBumper().onFalse(new InstantCommand(superstructure::unclimbEnd));
        
        transportSubsystem.setDefaultCommand(new TeleopIntake(() -> -operator.getLeftY()));
    
        operator.b().onTrue(new InstantCommand(() -> 
            transportSubsystem.setActive(!transportSubsystem.isActive())
        ));

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
            launcherSubsystem.changeLauncherSpeed(20)
        )));
        operator.povLeft().whileTrue(new RepeatCommand(new InstantCommand(() ->
            launcherSubsystem.changeLauncherSpeed(-20)
        )));
    }
}
