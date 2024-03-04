package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class RunClimberSideUntilStall extends Command {
    BooleanSupplier isStalling;
    DoubleConsumer setSideVelocity;

    public RunClimberSideUntilStall(DoubleConsumer setSideVelocity, BooleanSupplier isStalling) {
        addRequirements(Climber.getInstance());

        this.isStalling = isStalling;
        this.setSideVelocity = setSideVelocity;
    }

    @Override
    public boolean isFinished() {
        return isStalling.getAsBoolean();
    }

    @Override
    public void initialize() {
        setSideVelocity.accept(8);
    }

    @Override
    public void end(boolean inturrupted) {
        setSideVelocity.accept(0);
    }
}
