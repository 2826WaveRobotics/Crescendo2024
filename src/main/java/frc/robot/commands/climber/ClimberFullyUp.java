package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

/**
 * Moves the climber sides down until the motors stall. Resets the climber encoders once at the bottom.
 */
public class ClimberFullyUp extends ParallelCommandGroup {
    public ClimberFullyUp() {
        Climber climber = Climber.getInstance();
        addRequirements(climber);
        addCommands(
            new SetClimberSidePosition(climber::setLeftPosition, climber::getLeftPosition, Constants.Climber.fullUpRotations),
            new SetClimberSidePosition(climber::setRightPosition, climber::getRightPosition, Constants.Climber.fullUpRotations)
        );
    }
}
