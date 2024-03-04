package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Moves a climber side fully down based on methods to set the side position, set the side velocity, and get if the side is stalling.
 */
public class ClimberSideFullyDown extends SequentialCommandGroup {
    public ClimberSideFullyDown(
        DoubleConsumer setPosition,
        DoubleConsumer setVelocity,
        DoubleSupplier getPostion,
        BooleanSupplier sideStalling
    ) {
        addCommands(
            new SetClimberSidePosition(setPosition, getPostion, 0.3),
            new RunClimberSideUntilStall(setVelocity, sideStalling)
        );
    }
}
