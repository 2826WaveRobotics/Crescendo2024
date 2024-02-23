package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher;

public class TeleopLauncher extends Command {
    private Launcher launcherSubsystem;

    // Driver controller
    private Joystick operator;

    public TeleopLauncher(
            Launcher launcherSubsystem,
            Joystick operator) {
        this.launcherSubsystem = launcherSubsystem;
        addRequirements(launcherSubsystem);

        this.operator = operator;
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

        Trigger presetTrigger = new JoystickButton(operator, XboxController.Button.kY.value);
        presetTrigger.onTrue(new InstantCommand(() -> {
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(60));
            launcherSubsystem.launcherSpeed = 4000;
        }));
        
        Trigger presetTrigger2 = new JoystickButton(operator, XboxController.Button.kA.value);
        presetTrigger2.onTrue(new InstantCommand(() -> {
            launcherSubsystem.setLauncherAngle(Rotation2d.fromDegrees(39));
            launcherSubsystem.launcherSpeed = 4500;
        }));
    }

    @Override
    public void execute() {
        boolean invertLauncher = operator.getRawButton(XboxController.Button.kBack.value);
        launcherSubsystem.setLauncherInverted(invertLauncher);
    }
}
