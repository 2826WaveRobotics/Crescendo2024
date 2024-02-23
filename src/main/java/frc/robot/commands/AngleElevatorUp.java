package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

/**
 * Angles the elevator upward to the angle defined under `Constants.Elevator.elevatorUpAngle`.
 */
public class AngleElevatorUp extends Command {
    private Elevator elevatorSubsystem;

    public AngleElevatorUp(Elevator elevatorSubsystem) {
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getAngle() >= Constants.Elevator.elevatorUpAngle;
    }

    @Override
    public void initialize() {
        if(elevatorSubsystem.currentState != ElevatorState.AngledUp) {
            cancel();
            return;
        }
        
        elevatorSubsystem.setAngleSpeed(-Constants.Elevator.angleSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setExtensionSpeed(0);

        elevatorSubsystem.currentState = ElevatorState.Extended;
    }
}
