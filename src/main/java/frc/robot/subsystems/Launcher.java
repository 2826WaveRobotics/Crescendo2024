package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

// List class features here, including any motors, sensors, and functionality:
// 2 Launcher motors to score notes
// 1 motor to aim (~30-60 degree window)
// 1 CAN Coder for getting the absolute angle
public class Launcher extends SubsystemBase {
  // Declare member variables here
  private CANSparkMax topRollerMotor;
  private CANSparkMax bottomRollerMotor;
  private CANSparkMax angleLauncherMotor;

  private RelativeEncoder angleLauncherEncoder;
  private DutyCycleEncoder absoluteAngleLauncherEncoder;

  private final SparkPIDController topLaunchRollerPIDController;
  private final SparkPIDController bottomLaunchRollerPIDController;
  private final SparkPIDController anglePIDController;

  private final Joystick operatorController;
  private SlewRateLimiter launchVelocitySlewLimiter;
  
  double launcherAngle = 45;
  double launcherSpeed =  1200;
  boolean intakingNote = false;
  
  public Launcher(Joystick controller) {
    // Instantiate member variables and necessary code
    topRollerMotor = new CANSparkMax(Constants.Launcher.topRollerCANID, CANSparkMax.MotorType.kBrushless);
    bottomRollerMotor = new CANSparkMax(Constants.Launcher.bottomRollerCANID, CANSparkMax.MotorType.kBrushless);
    angleLauncherMotor = new CANSparkMax(Constants.Launcher.angleMotorCANID, CANSparkMax.MotorType.kBrushless);

    operatorController = controller;

    angleLauncherEncoder = angleLauncherMotor.getEncoder();
    absoluteAngleLauncherEncoder = new DutyCycleEncoder(0);

    topLaunchRollerPIDController = topRollerMotor.getPIDController();
    bottomLaunchRollerPIDController = bottomRollerMotor.getPIDController();

    anglePIDController = angleLauncherMotor.getPIDController();

    configMotorControllers();

    resetToAbsolute();

    launchVelocitySlewLimiter = new SlewRateLimiter(Constants.Launcher.launchVelocityRateLimit);

    setLauncherAngle(Rotation2d.fromDegrees(launcherAngle));
    final Trigger launcherUp = new Trigger(() -> {
      return operatorController.getPOV() == 0;
    });
    final Trigger launcherDown = new Trigger(() -> {
      return operatorController.getPOV() == 180;
    });
    launcherUp.onTrue(new InstantCommand(() -> {
      launcherAngle += 0.25;
      setLauncherAngle(Rotation2d.fromDegrees(launcherAngle));
      
      SmartDashboard.putNumber("LauncherAngle", launcherAngle);
    }));
    launcherDown.onTrue(new InstantCommand(() -> {
      launcherAngle -= 0.25;
      setLauncherAngle(Rotation2d.fromDegrees(launcherAngle));

      SmartDashboard.putNumber("LauncherAngle", launcherAngle);
    }));

    final Trigger speedUp = new Trigger(() -> {
      return operatorController.getPOV() == 90;
    });
    final Trigger speedDown = new Trigger(() -> {
      return operatorController.getPOV() == 270;
    });
    speedUp.onTrue(new InstantCommand(() -> {
      launcherSpeed += 50;
      
      SmartDashboard.putNumber("LauncherSpeed", launcherSpeed);
    }));
    speedDown.onTrue(new InstantCommand(() -> {
      launcherSpeed -= 50;

      SmartDashboard.putNumber("LauncherSpeed", launcherSpeed);
    }));
  }

  public double getAbsoluteLauncherAngleDegrees() {
    return absoluteAngleLauncherEncoder.getAbsolutePosition() * 360;
  }

  public double getLauncherAngleDegrees() {
    return (angleLauncherEncoder.getPosition() / Constants.Launcher.angleMotorGearboxReduction * 360.) % 360.;
  }

  public void resetToAbsolute() {
    double absolutePositionDegrees = (getAbsoluteLauncherAngleDegrees() - Constants.Launcher.angleOffset) % 360.;
    angleLauncherEncoder.setPosition(absolutePositionDegrees / 360. * Constants.Launcher.angleMotorGearboxReduction);
  }

  public void configMotorControllers() {
    topRollerMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(topRollerMotor, Usage.kPositionOnly);
    topRollerMotor.setSmartCurrentLimit(Constants.Launcher.rollerCurrentLimit);
    topRollerMotor.setIdleMode(Constants.Launcher.rollerIdleMode);
    topLaunchRollerPIDController.setP(Constants.Launcher.rollerKP);
    topLaunchRollerPIDController.setI(Constants.Launcher.rollerKI);
    topLaunchRollerPIDController.setD(Constants.Launcher.rollerKD);
    topLaunchRollerPIDController.setFF(Constants.Launcher.rollerKFF);
    topRollerMotor.enableVoltageCompensation(Constants.Launcher.voltageComp);
    topRollerMotor.burnFlash();
    
    bottomRollerMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(bottomRollerMotor, Usage.kPositionOnly);
    bottomRollerMotor.setSmartCurrentLimit(Constants.Launcher.rollerCurrentLimit);
    bottomRollerMotor.setIdleMode(Constants.Launcher.rollerIdleMode);
    bottomLaunchRollerPIDController.setP(Constants.Launcher.rollerKP);
    bottomLaunchRollerPIDController.setI(Constants.Launcher.rollerKI);
    bottomLaunchRollerPIDController.setD(Constants.Launcher.rollerKD);
    bottomLaunchRollerPIDController.setFF(Constants.Launcher.rollerKFF);
    bottomRollerMotor.enableVoltageCompensation(Constants.Launcher.voltageComp);
    bottomRollerMotor.burnFlash();

    angleLauncherMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleLauncherMotor, Usage.kPositionOnly);
    angleLauncherMotor.setSmartCurrentLimit(Constants.Launcher.angleCurrentLimit);
    angleLauncherMotor.setIdleMode(Constants.Launcher.angleIdleMode);
    anglePIDController.setP(Constants.Launcher.angleKP);
    anglePIDController.setI(Constants.Launcher.angleKI);
    anglePIDController.setD(Constants.Launcher.angleKD);
    anglePIDController.setFF(Constants.Launcher.angleKFF);
    angleLauncherMotor.enableVoltageCompensation(Constants.Launcher.voltageComp);
    angleLauncherMotor.setInverted(Constants.Launcher.invertAngle);
    angleLauncherMotor.burnFlash();

    this.launchRollersSlow();
  }

  /**
   * Calculates the required conch angle from the wanted launcher angle.
   * @param angle
   */
  public void setLauncherAngle(Rotation2d angle) {
    // All length units here are in inches
    double conchToPivotDistance = 5.153673;
    double pivotToConchReactionBarDistance = 4.907;
    double angleOffsetRadians = Units.degreesToRadians(30.826);
    // Law of cosines
    double requiredRadius = Math.sqrt(
      conchToPivotDistance * conchToPivotDistance + pivotToConchReactionBarDistance * pivotToConchReactionBarDistance
      - 2 * conchToPivotDistance * pivotToConchReactionBarDistance * Math.cos(Math.max(angle.getRadians() - angleOffsetRadians, 0.0))
    );
    
    double lowRadius = 0.1875; // in, from CAD
    double radiusIncrease = 3.4375; // in, from CAD: (4 -0.375/2-0.5/2)-(1/8)

    double conchAngleRadians = (requiredRadius - lowRadius) / radiusIncrease * (Math.PI * 2);
    if(conchAngleRadians < Constants.Launcher.softStopMarginLow.getRadians()) {
      // The angle is lower than we can achieve
      conchAngleRadians = Constants.Launcher.softStopMarginLow.getRadians();
    }
    if(conchAngleRadians > Math.PI * 2 - Constants.Launcher.softStopMarginHigh.getRadians()) {
      // The angle is higher than we can achieve
      conchAngleRadians = Math.PI * 2 - Constants.Launcher.softStopMarginHigh.getRadians();
    }

    anglePIDController.setReference(Rotation2d.fromRadians(conchAngleRadians).getRotations() * Constants.Launcher.angleMotorGearboxReduction, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Enables the launch rollers.
   */
  public void launchRollersFast() {
    launcherSpeed = Constants.Launcher.maxRollerVelocity;
  }

  /**
   * Disables the launch rollers.
   */
  public void launchRollersSlow() {
    launcherSpeed = Constants.Launcher.launchRollerVelocity;
  }

  /**
   * Use to take a Note in through the Launcher.
   */
  public void setLaunchNoteIn() {
    intakingNote = true;
  }

  public void setLaunchNoteOut() {
    intakingNote = false;
  }

  public void runLauncher() {
    double launchVelocity = launcherSpeed;

    if (intakingNote) {
      topLaunchRollerPIDController.setReference(-launchVelocity, CANSparkMax.ControlType.kVelocity);
      bottomLaunchRollerPIDController.setReference(-launchVelocity, CANSparkMax.ControlType.kVelocity);    
    } else {
      topLaunchRollerPIDController.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);
      bottomLaunchRollerPIDController.setReference(launchVelocity, CANSparkMax.ControlType.kVelocity);    
    }
  }

  @Override
  public void periodic() {

    runLauncher();
    // double launchVelocity = launchVelocitySlewLimiter.calculate(launchVelocityTarget);
    
    SmartDashboard.putNumber("Launcher absolute encoder", getAbsoluteLauncherAngleDegrees());
    SmartDashboard.putNumber("Launcher relative encoder", getLauncherAngleDegrees());
  }
}
