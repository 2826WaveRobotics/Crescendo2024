package frc.robot.commands.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorAngle extends Command {
    public Rotation2d targetAngle;

    public SetElevatorAngle(Rotation2d angle) {
        addRequirements(Elevator.getInstance());
        targetAngle = angle;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Elevator.getInstance().getAbsoluteAngle().minus(targetAngle).getDegrees()) <= 3;
    }

    @Override
    public void initialize() {
        Elevator.getInstance().setAnglePosition(targetAngle);
    }
}
