package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transport;

public class TeleopIntake extends Command {
    private Transport transportSubsystem;

    // Driver controller
    private BooleanSupplier bottomRollersRunning;
    private BooleanSupplier topRollersRunning;
    private BooleanSupplier reverseTransport;

    public TeleopIntake(
            Transport transportSubsystem,
            BooleanSupplier bottomRollersRunning,
            BooleanSupplier topRollersRunning,
            BooleanSupplier reverseTransport) {
        this.transportSubsystem = transportSubsystem;
        addRequirements(transportSubsystem);

        this.bottomRollersRunning = bottomRollersRunning;
        this.topRollersRunning = topRollersRunning;
        this.reverseTransport = reverseTransport;
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
    }

    @Override
    public void execute() {
        if(DriverStation.isAutonomous()) return;

        transportSubsystem.setIntakeSpeed(bottomRollersRunning.getAsBoolean() ? (reverseTransport.getAsBoolean() ? -10.0 : 10.0) : 0);
        transportSubsystem.setUpperTransportSpeed(topRollersRunning.getAsBoolean() ? (reverseTransport.getAsBoolean() ? -10.0 : 10.0) : 0);
    }
}
