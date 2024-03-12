package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;

public class RunClimberSideUntilStall extends Command {
    BooleanSupplier isStalling;
    DoubleConsumer setSideVelocity;

    public RunClimberSideUntilStall(DoubleConsumer setSideVelocity, BooleanSupplier isStalling) {
        this.isStalling = isStalling;
        this.setSideVelocity = setSideVelocity;
    }

    Debouncer debouncer = new Debouncer(0.25, DebounceType.kRising);

    @Override
    public boolean isFinished() {
        return debouncer.calculate(isStalling.getAsBoolean());
    }

    @Override
    public void initialize() {
        setSideVelocity.accept(2000);
    }

    @Override
    public void end(boolean inturrupted) {
        setSideVelocity.accept(0);
    }
}
