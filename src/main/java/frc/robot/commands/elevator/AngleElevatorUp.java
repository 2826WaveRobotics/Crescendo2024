package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

/**
 * Angles the elevator upward to the angle defined under `Constants.Elevator.elevatorUpAngle`.
 */
public class AngleElevatorUp extends SequentialCommandGroup {
    public AngleElevatorUp() {
        addCommands(new SetElevatorAngle(Constants.Elevator.elevatorUpAngle));
        finallyDo(() -> Elevator.getInstance().currentState = ElevatorState.Extended);
    }
}