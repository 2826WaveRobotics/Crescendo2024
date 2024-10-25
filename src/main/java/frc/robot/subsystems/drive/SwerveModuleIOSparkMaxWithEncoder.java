package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.subsystems.drive.EncoderIO.EncoderIOInputs;

import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO,
 * NEO 550, or NEO Vortex), and CANCoder absolute encoder.
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class SwerveModuleIOSparkMaxWithEncoder implements SwerveModuleIO {
  protected final CANSparkMax driveSparkMax;
  protected final CANSparkMax turnSparkMax;

  private final SparkPIDController drivePIDController;
  private final SparkPIDController turnPIDController;

  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public SwerveModuleIOSparkMaxWithEncoder(SwerveModuleConstants moduleConstants) {
    driveSparkMax = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    turnSparkMax = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);

    driveEncoder = driveSparkMax.getEncoder();
    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();

    drivePIDController = driveSparkMax.getPIDController();
    turnPIDController = turnSparkMax.getPIDController();

    Constants.Swerve.driveConfig.configure(driveSparkMax, drivePIDController, false, "swerve drive motor " + moduleConstants.driveMotorID);
    Constants.Swerve.angleConfig.configure(turnSparkMax, turnPIDController, false, "swerve angle motor " + moduleConstants.angleMotorID);

    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(0.0);
    turnPIDController.setPositionPIDWrappingMaxInput(Constants.Swerve.angleGearRatio);
    turnPIDController.setFeedbackDevice(turnAbsoluteEncoder);
    turnAbsoluteEncoder.setPositionConversionFactor(Constants.Swerve.angleGearRatio);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int)(1000.0 / SwerveModule.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int)(1000.0 / SwerveModule.ODOMETRY_FREQUENCY));

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();

    // Odometry update logic initialization
    
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();

    drivePositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(
      () -> {
        double value = driveEncoder.getPosition();
        if (driveSparkMax.getLastError() == REVLibError.kOk) {
          return OptionalDouble.of(value);
        } else {
          return OptionalDouble.empty();
        }
      }
    );
    turnPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(
      () -> {
        double value = turnAbsoluteEncoder.getPosition();
        if (driveSparkMax.getLastError() == REVLibError.kOk) {
          return OptionalDouble.of(value);
        } else {
          return OptionalDouble.empty();
        }
      }
    );
  }

  /** Resets the relative angle encoder to the CANCoder's absolute position. */
  @Override
  public void resetToAbsolute(EncoderIO encoder) {
    EncoderIOInputs inputs = new EncoderIOInputs();
    encoder.updateInputs(inputs);
    turnAbsoluteEncoder.setZeroOffset(inputs.absoluteTurnPosition.getRotations() * Constants.Swerve.angleGearRatio);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()) / Constants.Swerve.driveGearRatio;
    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / Constants.Swerve.driveGearRatio;
    inputs.driveCurrentAmps = driveSparkMax.getOutputCurrent();
    
    inputs.turnPosition = Rotation2d.fromRotations(turnAbsoluteEncoder.getPosition() / Constants.Swerve.angleGearRatio);
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnAbsoluteEncoder.getVelocity()) / Constants.Swerve.angleGearRatio;
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();

    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
      .mapToDouble((Double value) -> Units.rotationsToRadians(value) / Constants.Swerve.driveGearRatio)
      .toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
      .map((Double value) -> Rotation2d.fromRotations(value / Constants.Swerve.angleGearRatio))
      .toArray(Rotation2d[]::new);
    
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  double oldReferenceRPM = 0;
  @Override
  public void setDriveVelocity(double rpm) {
    if(rpm == oldReferenceRPM) return;
    oldReferenceRPM = rpm;

    drivePIDController.setReference(rpm * Constants.Swerve.driveGearRatio, ControlType.kVelocity);
  }

  double oldTurnAngle = 0;
  @Override
  public void setTurnAngle(Rotation2d angle) {
    if(angle.getRadians() == oldTurnAngle) return;
    oldTurnAngle = angle.getRadians();

    turnPIDController.setReference(angle.getRotations() * Constants.Swerve.angleGearRatio, ControlType.kPosition);
  }

  boolean oldDriveBrakeModeEnabled = false;
  @Override
  public void setDriveBrakeMode(boolean enable) {
    if(enable == oldDriveBrakeModeEnabled) return;
    oldDriveBrakeModeEnabled = enable;

    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  boolean oldTurnBrakeModeEnabled = false;
  @Override
  public void setTurnBrakeMode(boolean enable) {
    if(enable == oldTurnBrakeModeEnabled) return;
    oldTurnBrakeModeEnabled = enable;
    
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setCharacterizationDriveVoltage(double voltage) {
    driveSparkMax.setVoltage(voltage);
  }
}