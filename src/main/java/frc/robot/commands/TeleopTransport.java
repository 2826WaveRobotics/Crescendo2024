package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Transport;

public class TeleopTransport extends Command {
    private Transport transportSubsystem;

    // Driver Controller
    private Joystick driver;
    // Operator controller
    private Joystick operator;

    public TeleopTransport(
            Transport transportSubsystem,
            Joystick driver,
            Joystick operator) {
        this.transportSubsystem = transportSubsystem;
        addRequirements(transportSubsystem);

        this.driver = driver;
        this.operator = operator;
    }

    @Override
    public void initialize() {
        /**
         * A trigger for the upper transport toggle button, from 0 to 1. Gets the value from
         * the upper transport speed axis.
         */
        Trigger upperTransportTrigger  = new Trigger (() -> {
            return operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > Constants.Intake.intakeDeadband;
        });

        upperTransportTrigger.onTrue(new InstantCommand(() -> 
            transportSubsystem.upperTransportOn()
        ));

        upperTransportTrigger.onFalse(new InstantCommand(() -> 
            transportSubsystem.upperTransportOff()
        ));

        /**
         * A trigger for the lower transport toggle button, from 0 to 1. Gets the value from
         * the lower transport speed axis.
         */
        Trigger lowerTransportTrigger  = new Trigger (() -> {
            return driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > Constants.Intake.intakeDeadband;
        });

        lowerTransportTrigger.onTrue(new InstantCommand(() -> 
            transportSubsystem.lowerTransportOn()
        ));

        lowerTransportTrigger.onFalse(new InstantCommand(() -> 
            transportSubsystem.lowerTransportOff()
        ));

    }
}
