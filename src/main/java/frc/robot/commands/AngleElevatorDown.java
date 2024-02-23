package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class AngleElevatorDown extends Command {
    private Elevator elevatorSubsystem;

    public AngleElevatorDown(Elevator elevatorSubsystem) {
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.angleMotorIsStalling();
    }

    @Override
    public void initialize() {
        if(elevatorSubsystem.currentState == ElevatorState.Extended) {
            cancel();
            return;
        }

        elevatorSubsystem.setAngleSpeed(Constants.Elevator.angleSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setExtensionSpeed(0);
        elevatorSubsystem.resetAngleEncoder();

        elevatorSubsystem.currentState = ElevatorState.Stowed;
    }
}
