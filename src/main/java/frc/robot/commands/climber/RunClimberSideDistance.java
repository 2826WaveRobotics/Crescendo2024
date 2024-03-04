package frc.robot.commands.climber;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RunClimberSideDistance extends SequentialCommandGroup {
    public RunClimberSideDistance(DoubleConsumer setPosition, DoubleSupplier getPosition, double distance) {
        addCommands(
            new SetClimberSidePosition(setPosition, getPosition, Math.max(0, getPosition.getAsDouble() + distance))
        );
    }
}
