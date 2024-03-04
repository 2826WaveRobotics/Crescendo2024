package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class RunElevatorAngleUntilStall extends Command {
    public RunElevatorAngleUntilStall() {
        addRequirements(Elevator.getInstance());
    }

    @Override
    public boolean isFinished() {
        return Elevator.getInstance().angleMotorIsStalling();
    }

    @Override
    public void initialize() {
        Elevator.getInstance().setAngleSpeed(-5);
    }

    @Override
    public void end(boolean inturrupted) {
        Elevator.getInstance().setAngleSpeed(0);
    }
}
