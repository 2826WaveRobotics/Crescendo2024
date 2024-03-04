package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

/**
 * Runs the elevator downward until it hits the hard stop. Does nothing if the elevator is stowed.
 */
public class RetractElevator extends SequentialCommandGroup {
    private Elevator elevatorSubsystem;

    public RetractElevator() {
        addCommands(
            new SetElevatorExtensionDistance(0.05),
            new RunElevatorExtensionUntilStall()
        );

        finallyDo(() -> {
            if(elevatorSubsystem.currentState == ElevatorState.Extended)
                elevatorSubsystem.currentState = ElevatorState.AngledUp;
            else
                elevatorSubsystem.currentState = ElevatorState.Stowed;
        });
    }
}
