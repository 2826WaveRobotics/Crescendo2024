package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

// List class features here, including any motors, sensors, and functionality:
// Two Motors for the intake
// Primary motor has first contact with the Note.  Secondary controls the rollers behind the primary motor.
// This is the intake subsystem where the intake is fully internal in the robot
public class Intake extends SubsystemBase {
  // Declare member variables here
  private CANSparkMax primaryIntakeMotor;
  private CANSparkMax secondaryIntakeMotor;

  private SparkPIDController primaryIntakePIDController;
  private SparkPIDController secondaryIntakePIDController;

  public Intake() {
    // Instantiate member variables and necessary code
    primaryIntakeMotor = new CANSparkMax(51, CANSparkMax.MotorType.kBrushless);
    secondaryIntakeMotor = new CANSparkMax(52, CANSparkMax.MotorType.kBrushless);

    primaryIntakePIDController = primaryIntakeMotor.getPIDController();
    secondaryIntakePIDController = secondaryIntakeMotor.getPIDController();
    
    configMotorControllers();
  }

  
  public void configMotorControllers() {
    primaryIntakeMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(primaryIntakeMotor, Usage.kPositionOnly);
    primaryIntakeMotor.setSmartCurrentLimit(Constants.Intake.intakeCurrentLimit);
    primaryIntakeMotor.setIdleMode(Constants.Intake.intakeIdleMode);
    primaryIntakePIDController.setP(Constants.Intake.intakeKP);
    primaryIntakePIDController.setI(Constants.Intake.intakeKI);
    primaryIntakePIDController.setD(Constants.Intake.intakeKD);
    primaryIntakePIDController.setFF(Constants.Intake.intakeKFF);
    primaryIntakeMotor.enableVoltageCompensation(Constants.Intake.voltageComp);
    primaryIntakeMotor.burnFlash();
    
    secondaryIntakeMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(secondaryIntakeMotor, Usage.kPositionOnly);
    secondaryIntakeMotor.setSmartCurrentLimit(Constants.Intake.intakeCurrentLimit);
    secondaryIntakeMotor.setIdleMode(Constants.Intake.intakeIdleMode);
    secondaryIntakePIDController.setP(Constants.Intake.intakeKP);
    secondaryIntakePIDController.setI(Constants.Intake.intakeKI);
    secondaryIntakePIDController.setD(Constants.Intake.intakeKD);
    secondaryIntakePIDController.setFF(Constants.Intake.intakeKFF);
    secondaryIntakeMotor.enableVoltageCompensation(Constants.Intake.voltageComp);
    secondaryIntakeMotor.burnFlash();
  }

  /**
   * Sets the intake motors to the given speed.
   * @param speed The speed the the belt/edge of intake wheels will move at, in meters per second.
   */
  public void setIntakeSpeed(double speedInMetersPerSecond) {
    double beltPulleyCircumference = Constants.Intake.beltPulleyRadius * Math.PI * 2;
    double revolutionsPerSecond = speedInMetersPerSecond / beltPulleyCircumference;
    double rpm = revolutionsPerSecond * 60;

    primaryIntakePIDController.setReference(rpm, ControlType.kVelocity);
    secondaryIntakePIDController.setReference(rpm, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // Insert periodic code here
    
  }
}
