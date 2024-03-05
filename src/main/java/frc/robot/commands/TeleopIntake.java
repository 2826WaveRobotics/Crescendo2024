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

    @Override
    public void execute() {
        if(DriverStation.isAutonomous()) return;

        double intakeSpeed = MathUtil.applyDeadband(intakeOverride.getAsDouble(), 0.25);

        if(intakeSpeed != 0) {
            Transport.getInstance().setIntakeSpeed(intakeSpeed);
            Transport.getInstance().setUpperTransportSpeed(intakeSpeed);
        }
    }
}
