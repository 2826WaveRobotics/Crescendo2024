package frc.robot.commands.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

/**
 * Angles the elevator upward to the angle defined under `Constants.Elevator.elevatorUpAngle`.
 */
public class ElevatorCommands {
    public static Command angleElevatorUp() {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addRequirements(Elevator.getInstance());
        command.addCommands(new SetElevatorAngle(Constants.Elevator.elevatorUpAngle));
        return command.finallyDo(() -> Elevator.getInstance().currentState = ElevatorState.Extended);
    }

    public static Command angleElevatorDown() {
        if(Elevator.getInstance().currentState == ElevatorState.Extended) {
            return new InstantCommand(() -> {
                System.out.println("WARNING: Attempted to angle down elevator while extended");
            });
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addRequirements(Elevator.getInstance());

        command.addCommands(
            new SetElevatorAngle(Rotation2d.fromDegrees(5)),
            new RunElevatorAngleUntilStall()
        );

        return command.finallyDo(() -> Elevator.getInstance().currentState = ElevatorState.Stowed);
    }

    public static Command extendElevator() {
        if(Elevator.getInstance().currentState == ElevatorState.Stowed) {
            return new InstantCommand(() -> {
                System.out.println("WARNING: Attempted to extend elevator while stowed");
            });
        }

        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addRequirements(Elevator.getInstance());

        command.addCommands(new SetElevatorExtensionDistance(Constants.Elevator.elevatorExtendedHeight));

        return command.finallyDo(() -> {
            Elevator.getInstance().currentState = ElevatorState.Extended;
        });
    }

    public static Command retractElevator() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addRequirements(Elevator.getInstance());
        
        command.addCommands(
            new SetElevatorExtensionDistance(0.05),
            new RunElevatorExtensionUntilStall()
        );

        return command.finallyDo(() -> {
            Elevator elevatorSubsystem = Elevator.getInstance();
            if(elevatorSubsystem.currentState == ElevatorState.Extended)
                elevatorSubsystem.currentState = ElevatorState.AngledUp;
            else
                elevatorSubsystem.currentState = ElevatorState.Stowed;
        });
    }
}