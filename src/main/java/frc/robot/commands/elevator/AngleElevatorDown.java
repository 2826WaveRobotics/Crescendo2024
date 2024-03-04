package frc.robot.commands.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

/**
 * Angles the elevator upward to the angle defined under `Constants.Elevator.elevatorUpAngle`.
 */
public class AngleElevatorDown extends SequentialCommandGroup {
    public AngleElevatorDown() {
        if(Elevator.getInstance().currentState == ElevatorState.Extended) {
            cancel();
            return;
        }

        addCommands(
            new SetElevatorAngle(Rotation2d.fromDegrees(5)),
            new RunElevatorAngleUntilStall()
        );
        finallyDo(() -> Elevator.getInstance().currentState = ElevatorState.Extended);
    }
}