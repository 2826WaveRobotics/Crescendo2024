package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Launcher;

public class TeleopLauncher extends Command {
    private Launcher launcherSubsystem;

    private Trigger preset1Trigger;
    private Trigger preset2Trigger;
    private Trigger invertLauncher;

    private Trigger launcherUp;
    private Trigger launcherDown;
    private Trigger speedUp;
    private Trigger speedDown;

    public TeleopLauncher(
        Launcher launcherSubsystem,
        Trigger preset1Trigger,
        Trigger preset2Trigger,
        Trigger invertLauncher,
        Trigger launcherUp,
        Trigger launcherDown,
        Trigger speedUp,
        Trigger speedDown
    ) {
        this.launcherSubsystem = launcherSubsystem;
        addRequirements(launcherSubsystem);

        this.preset1Trigger = preset1Trigger;
        this.preset2Trigger = preset2Trigger;
        this.invertLauncher = invertLauncher;

        this.launcherUp = launcherUp;
        this.launcherDown = launcherDown;
        this.speedUp = speedUp;
        this.speedDown = speedDown;
    }

    @Override
    public void initialize() {
        // /**
        //  * A trigger for the intake toggle button, from 0 to 1. Gets the value from
        //  * the intake speed axis.
        //  */
        // Trigger intakeTrigger  = new Trigger (() -> {
        //     return driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > Constants.Intake.intakeDeadband;
        // });

        // intakeTrigger.onTrue(new InstantCommand(() -> 
        //     transportSubsystem.setActive(!transportSubsystem.isActive())
        // ));

        preset1Trigger.onTrue(new InstantCommand(() -> {
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(60));
            launcherSubsystem.launcherSpeed = 4000;
        }));
        preset2Trigger.onTrue(new InstantCommand(() -> {
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(39));
            launcherSubsystem.launcherSpeed = 4500;
        }));

        launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(45));
        launcherUp.whileTrue(new RepeatCommand(new InstantCommand(() -> {
            double launcherAngle = launcherSubsystem.getLauncherConchAngleDegrees() + 0.15;
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(launcherAngle));
        })));
        launcherDown.whileTrue(new RepeatCommand(new InstantCommand(() -> {
            double launcherAngle = launcherSubsystem.getLauncherConchAngleDegrees() - 0.15;
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(launcherAngle));
        })));

        speedUp.whileTrue(new RepeatCommand(new InstantCommand(() ->
            launcherSubsystem.changeLauncherSpeed(20)
        )));
        speedDown.whileTrue(new RepeatCommand(new InstantCommand(() ->
            launcherSubsystem.changeLauncherSpeed(-20)
        )));
    }

    @Override
    public void execute() {
        launcherSubsystem.setLauncherInverted(invertLauncher.getAsBoolean());
    }
}
