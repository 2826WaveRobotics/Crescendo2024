package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command {
  private Intake intakeSubsystem;
  
  /**
   * A double supplier for the intake's speed, from 0 to 1. Gets the value from the intake speed axis.
   */
  private DoubleSupplier intakeSpeedSupplier;

  /**
   * Limits the rate of change of the intake speed.
   */
  private SlewRateLimiter intakeSpeedLimiter = new SlewRateLimiter(3.0);

  public TeleopIntake(
      Intake intakeSubsystem,
      DoubleSupplier intakeSpeedSupplier) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);

    this.intakeSpeedSupplier = intakeSpeedSupplier;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double intakeSpeed = intakeSpeedLimiter.calculate(MathUtil.applyDeadband(intakeSpeedSupplier.getAsDouble(), Constants.Intake.intakeDeadband));

    intakeSubsystem.setIntakeSpeed(intakeSpeed * Constants.Intake.maximumIntakeSpeed);
  }
}
