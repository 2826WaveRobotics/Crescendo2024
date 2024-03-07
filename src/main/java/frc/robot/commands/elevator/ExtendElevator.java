package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

/**
 * Runs the elevator upward to the target elevator angle.
 */
public class ExtendElevator extends SequentialCommandGroup {
    public ExtendElevator() {
        addRequirements(Elevator.getInstance());
        
        if(Elevator.getInstance().currentState == ElevatorState.Stowed) {
            cancel();
            return;
        }

        addCommands(new SetElevatorExtensionDistance(Constants.Elevator.elevatorExtendedHeight));
        finallyDo(() -> {
            Elevator.getInstance().currentState = ElevatorState.Extended;
        });
    }
}
