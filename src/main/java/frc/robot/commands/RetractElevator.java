package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

/**
 * Runs the elevator downward until it hits the hard stop. Does nothing if the elevator is stowed.
 */
public class RetractElevator extends Command {
    private Elevator elevatorSubsystem;

    public RetractElevator(Elevator elevatorSubsystem) {
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.extensionMotorIsStalling();
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setExtensionSpeed(10);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setExtensionSpeed(0);
        elevatorSubsystem.resetExtensionEncoder();

        if(elevatorSubsystem.currentState == ElevatorState.Extended)
            elevatorSubsystem.currentState = ElevatorState.AngledUp;
        else
            elevatorSubsystem.currentState = ElevatorState.Stowed;
    }
}
