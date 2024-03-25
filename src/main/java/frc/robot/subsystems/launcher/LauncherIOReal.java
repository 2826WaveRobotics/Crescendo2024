package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class LauncherIOReal implements LauncherIO {
  private CANSparkMax topRollerMotor;
  private CANSparkMax bottomRollerMotor;
  private CANSparkMax angleLauncherMotor;

  private RelativeEncoder angleLauncherEncoder;
  private DutyCycleEncoder absoluteAngleLauncherEncoder;

  private final SparkPIDController topLaunchRollerPIDController;
  private final SparkPIDController bottomLaunchRollerPIDController;
  private final SparkPIDController anglePIDController;

  private final RelativeEncoder topRollerEncoder;

  public LauncherIOReal() {
    // Instantiate member variables and necessary code
    topRollerMotor = new CANSparkMax(Constants.Launcher.topRollerCANID, CANSparkMax.MotorType.kBrushless);
    bottomRollerMotor = new CANSparkMax(Constants.Launcher.bottomRollerCANID, CANSparkMax.MotorType.kBrushless);
    angleLauncherMotor = new CANSparkMax(Constants.Launcher.angleMotorCANID, CANSparkMax.MotorType.kBrushless);

    topLaunchRollerPIDController = topRollerMotor.getPIDController();
    bottomLaunchRollerPIDController = bottomRollerMotor.getPIDController();

    anglePIDController = angleLauncherMotor.getPIDController();

    Constants.Launcher.rollerConfig.configure(topRollerMotor, topLaunchRollerPIDController);
    Constants.Launcher.rollerConfig.configure(bottomRollerMotor, bottomLaunchRollerPIDController);
    
    angleLauncherMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleLauncherMotor, Usage.kPositionOnly);
    angleLauncherMotor.setSmartCurrentLimit(15);
    angleLauncherMotor.setIdleMode(IdleMode.kBrake);
    anglePIDController.setP(0.1);
    anglePIDController.setI(0.0);
    anglePIDController.setD(0.0);
    anglePIDController.setFF(0.000175);
    angleLauncherMotor.enableVoltageCompensation(12.0);
    angleLauncherMotor.setInverted(Constants.Launcher.invertAngle);

    // This doesn't work for some reason.
    Constants.Launcher.angleConfig.configure(angleLauncherMotor, anglePIDController);
    angleLauncherMotor.setSoftLimit(
      SoftLimitDirection.kReverse,
      (float)(Constants.Launcher.softStopMarginLow.getRotations() * Constants.Launcher.angleMotorGearboxReduction)
    );
    angleLauncherMotor.setSoftLimit(
      SoftLimitDirection.kForward,
      (float)((1 - Constants.Launcher.softStopMarginHigh.getRotations()) * Constants.Launcher.angleMotorGearboxReduction)
    );
    angleLauncherMotor.setInverted(Constants.Launcher.invertAngle);

    angleLauncherMotor.burnFlash();
    
    angleLauncherEncoder = angleLauncherMotor.getEncoder();
    absoluteAngleLauncherEncoder = new DutyCycleEncoder(Constants.Launcher.absoluteEncoderDIOPort);

    topRollerEncoder = topRollerMotor.getEncoder();

    resetToAbsolute();
  }
  
  public void resetToAbsolute() {
    Rotation2d absolutePosition = getAbsoluteLauncherAngle().minus(Constants.Launcher.angleOffset);
    double rotations = absolutePosition.getRotations() % 1.;
    if(rotations < 0.) rotations = rotations + 1;
    angleLauncherEncoder.setPosition(rotations * Constants.Launcher.angleMotorGearboxReduction);
  }

  private Rotation2d getAbsoluteLauncherAngle() {
    return Rotation2d.fromRotations(absoluteAngleLauncherEncoder.getAbsolutePosition());
  }

  private Rotation2d getLauncherConchAngle() {
    return Rotation2d.fromRotations((angleLauncherEncoder.getPosition() / Constants.Launcher.angleMotorGearboxReduction) % 1.);
  }

  @Override
  public void setAngleReference(double rotations) {
    if(Constants.enableNonEssentialShuffleboard) {
      SmartDashboard.putNumber("Launcher angle reference", rotations * 360.);
    }
    anglePIDController.setReference(rotations * Constants.Launcher.angleMotorGearboxReduction, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.absoluteLauncherAngle = getAbsoluteLauncherAngle();
    inputs.launcherRelativeConchAngle = getLauncherConchAngle();
    inputs.launcherAngleVeocityRPM = angleLauncherEncoder.getVelocity();
    inputs.speedRPM = topRollerEncoder.getVelocity();
  }

  @Override
  public void runRollers(double topRollerSpeed, double bottomRollerSpeed) {
    topLaunchRollerPIDController.setReference(topRollerSpeed, CANSparkMax.ControlType.kVelocity);
    bottomLaunchRollerPIDController.setReference(bottomRollerSpeed, CANSparkMax.ControlType.kVelocity);
  }
}