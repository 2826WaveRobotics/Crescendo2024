package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorExtensionDistance extends Command {
    private double targetDistance;

    public SetElevatorExtensionDistance(double distance) {
        addRequirements(Elevator.getInstance());

        targetDistance = distance;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Elevator.getInstance().getExtensionHeight() - targetDistance) < 0.05;
    }

    @Override
    public void initialize() {
        Elevator.getInstance().setExtensionPosition(targetDistance);
    }
}
