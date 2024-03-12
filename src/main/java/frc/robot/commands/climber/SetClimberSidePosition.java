package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Sets the position of a climber side.  
 * Positive positions are upward.  
 * Also stops when the motor is stalling.
 */
public class SetClimberSidePosition extends Command {
    double targetPosition;
    DoubleSupplier getPosition;
    BooleanSupplier isStalling;

    public SetClimberSidePosition(
        DoubleConsumer setPosition,
        DoubleSupplier getPosition,
        BooleanSupplier isStalling,
        double position
    ) {
      setPosition.accept(position);
      
      targetPosition = position;
      this.getPosition = getPosition;
      this.isStalling = isStalling;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getPosition.getAsDouble() - targetPosition) < 0.3 || isStalling.getAsBoolean();
    }
}
