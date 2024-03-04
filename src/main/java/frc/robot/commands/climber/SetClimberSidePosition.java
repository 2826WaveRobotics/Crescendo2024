package frc.robot.commands.climber;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class SetClimberSidePosition extends Command {
    double targetPosition;
    DoubleSupplier getPosition;

    public SetClimberSidePosition(
        DoubleConsumer setPosition,
        DoubleSupplier getPosition,
        double position
    ) {
      addRequirements(Climber.getInstance());
      
      setPosition.accept(position);
      
      targetPosition = position;
      this.getPosition = getPosition;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getPosition.getAsDouble() - targetPosition) < 0.3;
    }
}
