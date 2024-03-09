package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ClimberIOReal implements ClimberIO {
  private CANSparkMax leftClimberMotor;
  private CANSparkMax rightClimberMotor;

  private RelativeEncoder leftClimberEncoder;
  private RelativeEncoder rightClimberEncoder;

  private SparkPIDController leftPIDController;
  private SparkPIDController rightPIDController;

  public ClimberIOReal() {
    leftClimberMotor = new CANSparkMax(Constants.Climber.leftClimberMotorCANID, CANSparkMax.MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(Constants.Climber.rightClimberMotorCANID, CANSparkMax.MotorType.kBrushless);
    
    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

    leftPIDController = leftClimberMotor.getPIDController();
    rightPIDController = rightClimberMotor.getPIDController();
  }
  
  /** Resets the encoder value for the left motor to 0. */
  @Override
  public void resetLeftEncoder() {
    leftClimberEncoder.setPosition(0);
  }

  /** Resets the encoder value for the right motor to 0. */
  @Override
  public void resetRightEncoder() {
    rightClimberEncoder.setPosition(0);
  }
  
  /**
   * Sets the left motor speed in RPM. Positive values move the elevator downward.
   * @param speed The target speed, in RPM.
   */
  @Override
  public void setLeftSpeed(double speed) {
    leftPIDController.setReference(speed, ControlType.kVelocity, 1);
  }
  /**
   * Sets the right motor speed in RPM. Positive values move the elevator downward.
   * @param speed
   */
  @Override
  public void setRightSpeed(double speed) {
    rightPIDController.setReference(speed, ControlType.kVelocity, 1);
  }
  /**
   * Sets the right motor position in rotations, where 0 is the resting position and positive numbers are upward.
   * @param position
   */
  @Override
  public void setRightPosition(double position) {
    rightPIDController.setReference(position, ControlType.kPosition, 0);
  }
  /**
   * Sets the left motor position in rotations, where 0 is the resting position and positive numbers are upward.
   * @param position
   */
  @Override
  public void setLeftPosition(double position) {
    leftPIDController.setReference(position, ControlType.kPosition, 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberCurrentDrawAmps = leftClimberMotor.getOutputCurrent();
    inputs.rightClimberCurrentDrawAmps = rightClimberMotor.getOutputCurrent();

    inputs.leftPosition = leftClimberEncoder.getPosition();
    inputs.rightPosition = rightClimberEncoder.getPosition();
  }

  @Override
  public void useCurrentLimits(int smartLimit, int secondaryLimit) {
    leftClimberMotor.setSmartCurrentLimit(smartLimit);
    rightClimberMotor.setSmartCurrentLimit(smartLimit);
    leftClimberMotor.setSecondaryCurrentLimit(secondaryLimit);
    rightClimberMotor.setSecondaryCurrentLimit(secondaryLimit);
  }
}
