package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.transport.Transport;

public class TeleopIntake extends Command {
    private DoubleSupplier intakeOverride;

    public TeleopIntake(DoubleSupplier intakeOverride) {
        addRequirements(Transport.getInstance());

        this.intakeOverride = intakeOverride;
    }

    private double lastSetSpeed = 0;

    @Override
    public void execute() {
        if(DriverStation.isAutonomous()) return;

        double intakeSpeed = MathUtil.applyDeadband(intakeOverride.getAsDouble(), 0.25);

        if(lastSetSpeed != intakeSpeed) {
            lastSetSpeed = intakeSpeed;
            Transport.getInstance().setIntakeSpeed(intakeSpeed);
            Transport.getInstance().setUpperTransportSpeed(intakeSpeed);
        }
    }
}
