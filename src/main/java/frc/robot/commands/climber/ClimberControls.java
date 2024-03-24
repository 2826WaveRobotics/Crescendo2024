package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;

public class ClimberControls extends InstantCommand {
    public ClimberControls(
        BooleanSupplier moveLeft,
        BooleanSupplier moveRight,
        BooleanSupplier invertControls
    ) {
        super(() -> {
            if(!DriverStation.isTeleop()) return;
            
            double speed = invertControls.getAsBoolean() ? -Constants.Climber.climberMotorSpeed : Constants.Climber.climberMotorSpeed;

            Command scheduledClimbCommand = Superstructure.getInstance().scheduledClimbCommand;
            if(
                (moveLeft.getAsBoolean() || moveRight.getAsBoolean()) &&
                scheduledClimbCommand != null && scheduledClimbCommand.isScheduled()
            ) scheduledClimbCommand.cancel();

            Climber climber = Climber.getInstance();
            climber.setLeftSpeed ((moveLeft .getAsBoolean() ? 1 : 0) * speed);
            climber.setRightSpeed((moveRight.getAsBoolean() ? 1 : 0) * speed);
        });
    }
}
