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
  private CANSparkMax frontIntakeMotor;
  private CANSparkMax backIntakeMotor;
  private CANSparkMax beltIntakeMotor;

  private SparkPIDController frontIntakePIDController;
  private SparkPIDController backIntakePIDController;
  private SparkPIDController beltIntakePIDController;

  public Intake() {
    // Instantiate member variables and necessary code
    frontIntakeMotor = new CANSparkMax(Constants.Intake.frontIntakeMotorCANID, CANSparkMax.MotorType.kBrushless);
    // backIntakeMotor = new CANSparkMax(Constants.Intake.backIntakeMotorCANID, CANSparkMax.MotorType.kBrushless); // FIXME - only 2 motors
    beltIntakeMotor = new CANSparkMax(Constants.Intake.beltIntakeMotorCANID, CANSparkMax.MotorType.kBrushless);

    frontIntakePIDController = frontIntakeMotor.getPIDController();
    // backIntakePIDController = backIntakeMotor.getPIDController();
    beltIntakePIDController = beltIntakeMotor.getPIDController();
    
    configMotorControllers();
  }

  
  public void configMotorControllers() {
    frontIntakeMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(frontIntakeMotor, Usage.kPositionOnly);
    frontIntakeMotor.setSmartCurrentLimit(Constants.Intake.intakeCurrentLimit);
    frontIntakeMotor.setIdleMode(Constants.Intake.intakeIdleMode);
    frontIntakePIDController.setP(Constants.Intake.intakeKP);
    frontIntakePIDController.setI(Constants.Intake.intakeKI);
    frontIntakePIDController.setD(Constants.Intake.intakeKD);
    frontIntakePIDController.setFF(Constants.Intake.intakeKFF);
    frontIntakeMotor.enableVoltageCompensation(Constants.Intake.voltageComp);
    frontIntakeMotor.burnFlash();
    
    // backIntakeMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(backIntakeMotor, Usage.kPositionOnly);
    // backIntakeMotor.setSmartCurrentLimit(Constants.Intake.intakeCurrentLimit);
    // backIntakeMotor.setIdleMode(Constants.Intake.intakeIdleMode);
    // backIntakePIDController.setP(Constants.Intake.intakeKP);
    // backIntakePIDController.setI(Constants.Intake.intakeKI);
    // backIntakePIDController.setD(Constants.Intake.intakeKD);
    // backIntakePIDController.setFF(Constants.Intake.intakeKFF);
    // backIntakeMotor.enableVoltageCompensation(Constants.Intake.voltageComp);
    // backIntakeMotor.burnFlash();
    
    beltIntakeMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(beltIntakeMotor, Usage.kPositionOnly);
    beltIntakeMotor.setSmartCurrentLimit(Constants.Intake.intakeCurrentLimit);
    beltIntakeMotor.setIdleMode(Constants.Intake.intakeIdleMode);
    beltIntakePIDController.setP(Constants.Intake.intakeKP);
    beltIntakePIDController.setI(Constants.Intake.intakeKI);
    beltIntakePIDController.setD(Constants.Intake.intakeKD);
    beltIntakePIDController.setFF(Constants.Intake.intakeKFF);
    beltIntakeMotor.enableVoltageCompensation(Constants.Intake.voltageComp);
    beltIntakeMotor.burnFlash();
  }

  double desiredSpeedMetersPerSecond = 0;
  /**
   * If we're currently ejecting a note.
   * This happens when we accidentally intake two notes.
   * When this is true, the front and back intake rollers
   * run quickly in reverse to eject the note.
   */
  boolean isEjectingNote = false;

  /**
   * Returns if the intake is currently active, meaning the speed is nonzero.
   * @return
   */
  public boolean isActive() {
    return desiredSpeedMetersPerSecond != 0;
  }

  /**
   * Sets the intake active. False sets the speed to 0 and true sets the speed to `Constants.Intake.intakeSpeed`.
   * @param active
   */
  public void setActive(boolean active) {
    setIntakeSpeed(active ? Constants.Intake.intakeSpeed : 0);
  }

  /**
   * Sets the intake motors to the given speed.
   * @param speed The speed the the belt/edge of intake wheels will move at, in meters per second.
   */
  private void setIntakeSpeed(double speedInMetersPerSecond) {
    desiredSpeedMetersPerSecond = speedInMetersPerSecond;
  }

  /**
   * TESTING - TURN INTAKE ON
   */
  public void intakeOn() {
    frontIntakePIDController.setReference(5500, ControlType.kVelocity);
    beltIntakePIDController.setReference(5500, ControlType.kVelocity); //from -5500, changed to 3000 for testing - Ben
  }

  /**
   * TESTING - TURN INTAKE OFF
   */
  public void intakeOff() {
    frontIntakePIDController.setReference(0, ControlType.kVelocity);
    beltIntakePIDController.setReference(0, ControlType.kVelocity);
  }

  /**
   * Begin ejecting the note currently in the intake.  
   * This happens when we accidentally intake two notes.
   */
  public void ejectNote() {
    isEjectingNote = true;
  }

  /**
   * Begin ejecting the note currently in the intake.  
   * This happens when we accidentally intake two notes.
   */
  public void stopEjectingNote() {
    isEjectingNote = false;
  }

  @Override
  public void periodic() {
    // TESTING - REMOVING LOGIC
    // double speedMetersPerSecond = isEjectingNote ? -Constants.Intake.ejectSpeedMetersPerSecond : desiredSpeedMetersPerSecond;
    
    // double beltPulleyCircumference = Constants.Intake.beltPulleyRadius * Math.PI * 2;
    // double revolutionsPerSecond = speedMetersPerSecond / beltPulleyCircumference;
    // double rpm = revolutionsPerSecond * 60;

    // frontIntakePIDController.setReference(rpm, ControlType.kVelocity);
    // // backIntakePIDController.setReference(-rpm, ControlType.kVelocity); // FIXME - Only 2 motors
    // beltIntakePIDController.setReference(-rpm, ControlType.kVelocity);
  }
}
