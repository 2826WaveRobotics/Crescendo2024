package frc.robot.subsystems.transport;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;

public class TransportIOReal implements TransportIO {
  private CANSparkMax frontIntakeMotor;
  private CANSparkMax lowerTransportMotor;
  private CANSparkMax backIntakeMotor;
  private CANSparkMax upperTransportMotor;

  private SparkPIDController upperTransportPIDController;
  private SparkPIDController frontIntakePIDController;
  private SparkPIDController lowerTransportPIDController;
  private SparkPIDController backIntakePIDController;

  private RelativeEncoder backIntakeMotorEncoder;
  private RelativeEncoder frontIntakeMotorEncoder;

  public TransportIOReal() {
    frontIntakeMotor = new CANSparkMax(Constants.Intake.frontIntakeMotorCANID, CANSparkMax.MotorType.kBrushless);
    lowerTransportMotor = new CANSparkMax(Constants.Transport.lowerTransportMotorCANID, CANSparkMax.MotorType.kBrushless);
    backIntakeMotor = new CANSparkMax(Constants.Intake.backIntakeMotorCANID, CANSparkMax.MotorType.kBrushless);
    upperTransportMotor = new CANSparkMax(Constants.Transport.upperTransportMotorCANID, CANSparkMax.MotorType.kBrushless);

    frontIntakePIDController = frontIntakeMotor.getPIDController();
    lowerTransportPIDController = lowerTransportMotor.getPIDController();
    backIntakePIDController = backIntakeMotor.getPIDController();
    upperTransportPIDController = upperTransportMotor.getPIDController();

    Constants.Intake.intakeMotorConfig.configure(backIntakeMotor, backIntakePIDController);
    Constants.Intake.intakeMotorConfig.configure(frontIntakeMotor, frontIntakePIDController);

    Constants.Transport.transportMotorConfig.configure(lowerTransportMotor, lowerTransportPIDController);
    Constants.Transport.transportMotorConfig.configure(upperTransportMotor, upperTransportPIDController);

    frontIntakeMotorEncoder = frontIntakeMotor.getEncoder();
    backIntakeMotorEncoder = backIntakeMotor.getEncoder();

    if(!Constants.enableNonEssentialShuffleboard) return;
    Shuffleboard.getTab("Notes").addNumber("Bottom transport current draw", () -> lowerTransportMotor.getOutputCurrent());
  }

  /**
   * Gets the rpm that an intake motor should run at, based on the target speed of the edge of the wheels and belt.  
   * This is effectively the speed that the notes move at.
   * @param speedMetersPerSecond The speed the the belt/edge of intake wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.
   * @return
   */
  private double getIntakeMotorRPM(double speedMetersPerSecond, double gearRatio) {
    double wheelCircumference = Constants.Intake.wheelRadius * Math.PI * 2;
    double revolutionsPerSecond = speedMetersPerSecond / wheelCircumference * gearRatio;
    return revolutionsPerSecond * 60;
  }

  /**
   * The slew rate limiter for the transport speed.
   */
  private SlewRateLimiter transportSlewRateLimiter = new SlewRateLimiter(11000 * 6.);

  /**
   * Sets the transport speeds. Speeds are how fast the belt/edge of intake wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.  
   * If the intake is running faster than the transport, the transport will run at the intake speed.
   * @param transportSpeedMetersPerSecond The speed that the upper transport wheels will spin.
   * @param intakeSpeedMetersPerSecond The speed that the intake wheels will spin.
   */
  @Override
  public void setTransportSpeed(double transportSpeedMetersPerSecond, double intakeSpeedMetersPerSecond) {
    double limitedTransportSpeed = transportSlewRateLimiter.calculate(transportSpeedMetersPerSecond);
    if(transportSpeedMetersPerSecond == 0.0) limitedTransportSpeed = 0.0;

    double double5GearRaio = 5.23 * 5.23;
    double fiveAndThreeGearRaio = 5.23 * 2.89;

    double minSpeed = (intakeSpeedMetersPerSecond != 0) ? 400 : 0;

    // double frontIntakeRPM = Math.signum(intakeSpeedMetersPerSecond) * Math.max(Math.min(
    //   getIntakeMotorRPM(Math.abs(intakeSpeedMetersPerSecond), double5GearRaio),
    //   backIntakeMotorEncoder.getVelocity() / fiveAndThreeGearRaio * double5GearRaio
    // ), minSpeed);
    
    double frontIntakeRPM = getIntakeMotorRPM(intakeSpeedMetersPerSecond, double5GearRaio);
    frontIntakePIDController.setReference(frontIntakeRPM, ControlType.kVelocity);

    double backIntakeRPM = Math.signum(intakeSpeedMetersPerSecond) * Math.max(Math.min(
      getIntakeMotorRPM(Math.abs(intakeSpeedMetersPerSecond), fiveAndThreeGearRaio),
      frontIntakeMotorEncoder.getVelocity() / double5GearRaio * fiveAndThreeGearRaio
    ), minSpeed);
    backIntakePIDController.setReference(backIntakeRPM, ControlType.kVelocity);

    double lowerTransportRPM = getIntakeMotorRPM(limitedTransportSpeed, fiveAndThreeGearRaio);
    lowerTransportPIDController.setReference(-lowerTransportRPM, ControlType.kVelocity);

    double upperTransportSpeed = getIntakeMotorRPM(limitedTransportSpeed, double5GearRaio);
    upperTransportPIDController.setReference(-upperTransportSpeed, ControlType.kVelocity);
  }
}
