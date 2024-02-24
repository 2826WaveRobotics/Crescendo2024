package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.oi.ShuffleboardContent;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder absoluteAngleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
  
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    absoluteAngleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, CANSparkMax.MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    Constants.Swerve.angleConfig.configure(angleMotor, angleController);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, CANSparkMax.MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    Constants.Swerve.driveConfig.configure(driveMotor, driveController);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveEncoder.setPosition(0.0);
    
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);

    lastAngle = getState().angle;

    /* Shuffleboard Content for each swerve module */ 
    ShuffleboardContent.initSwerveModuleShuffleboard(this);

  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void resetToAbsolute() {
    double absolutePositionDegrees = getAbsoluteModuleAngleDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePositionDegrees);
  }

  private void configAngleEncoder() {
    absoluteAngleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    CANCoderUtil.setCANCoderBusUsage(absoluteAngleEncoder, CCUsage.kMinimal);
    absoluteAngleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          CANSparkMax.ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  public double getRelativeAngle() {
    return integratedAngleEncoder.getPosition();
  }

  public double getAbsoluteModuleAngleDegrees() {
    return absoluteAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360.;
  }

  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCANcoderAbsoluteAngle() {
    return Rotation2d.fromDegrees(getAbsoluteModuleAngleDegrees());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  // returns the current position as SwerveModulePosition
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      driveEncoder.getPosition(), Rotation2d.fromDegrees(integratedAngleEncoder.getPosition())
    );
  }

  public String getModuleName(int id) {
    return Constants.Swerve.moduleNames[id];
  }

  public double getDriveMotorCurrent() {
    return driveMotor.getOutputCurrent();
  }

  public double getTurnMotorCurrent() {
    return angleMotor.getOutputCurrent();
  }

  public double getOffset() {
    return angleOffset.getDegrees();
  }

  public StatusSignal<MagnetHealthValue> getMagnetFieldStrength() {
    return absoluteAngleEncoder.getMagnetHealth();
  }

}
