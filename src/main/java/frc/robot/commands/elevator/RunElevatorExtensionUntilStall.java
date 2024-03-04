package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class RunElevatorExtensionUntilStall extends Command {
    public RunElevatorExtensionUntilStall() {
        addRequirements(Elevator.getInstance());
    }

    @Override
    public boolean isFinished() {
        return Elevator.getInstance().extensionMotorIsStalling();
    }

    @Override
    public void initialize() {
        Elevator.getInstance().setExtensionSpeed(5);
    }

    @Override
    public void end(boolean inturrupted) {
        Elevator.getInstance().setExtensionSpeed(0);
    }
}
