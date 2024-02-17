package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

/**
 * Runs the elevator upward to the target elevator angle.
 */
public class ExtendElevator extends Command {
    private Elevator elevatorSubsystem;

    public ExtendElevator(Elevator elevatorSubsystem) {
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getExtensionHeight() >= Constants.Elevator.elevatorExtendedHeight;
    }

    @Override
    public void initialize() {
        if(elevatorSubsystem.currentState != ElevatorState.AngledUp) cancel();
        
        elevatorSubsystem.setExtensionSpeed(-10);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setExtensionSpeed(0);

        elevatorSubsystem.currentState = ElevatorState.Extended;
    }
}
