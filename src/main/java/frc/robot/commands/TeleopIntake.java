package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Transport;

public class TeleopIntake extends Command {
    private Transport transportSubsystem;

    // Driver controller
    private Joystick driver;

    public TeleopIntake(
            Transport transportSubsystem,
            Joystick driver) {
        this.transportSubsystem = transportSubsystem;
        addRequirements(transportSubsystem);

        this.driver = driver;
    }

    @Override
    public void initialize() {
        /**
         * A trigger for the intake toggle button, from 0 to 1. Gets the value from
         * the intake speed axis.
         */
        Trigger intakeTrigger  = new Trigger (() -> {
            return driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > Constants.Intake.intakeDeadband;
        });

        intakeTrigger.onTrue(new InstantCommand(() -> 
            transportSubsystem.setActive(!transportSubsystem.isActive())
        ));
    }
}
