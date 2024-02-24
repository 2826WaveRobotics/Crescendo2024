package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
  
  boolean intakingNote = false;
  
  public Launcher() {
    // Instantiate member variables and necessary code
    topRollerMotor = new CANSparkMax(Constants.Launcher.topRollerCANID, CANSparkMax.MotorType.kBrushless);
    bottomRollerMotor = new CANSparkMax(Constants.Launcher.bottomRollerCANID, CANSparkMax.MotorType.kBrushless);
    angleLauncherMotor = new CANSparkMax(Constants.Launcher.angleMotorCANID, CANSparkMax.MotorType.kBrushless);

    angleLauncherEncoder = angleLauncherMotor.getEncoder();
    absoluteAngleLauncherEncoder = new DutyCycleEncoder(0);

    topLaunchRollerPIDController = topRollerMotor.getPIDController();
    bottomLaunchRollerPIDController = bottomRollerMotor.getPIDController();

    anglePIDController = angleLauncherMotor.getPIDController();

    Constants.Launcher.rollerConfig.configure(topRollerMotor, topLaunchRollerPIDController);
    Constants.Launcher.rollerConfig.configure(bottomRollerMotor, bottomLaunchRollerPIDController);
    Constants.Launcher.angleConfig.configure(angleLauncherMotor, anglePIDController);
    angleLauncherMotor.setInverted(Constants.Launcher.invertAngle);
    // TODO: Angle motor soft limits in the SPARK
    launchRollersSlow();

    resetToAbsolute();
  }

  public double getAbsoluteLauncherAngleDegrees() {
    return absoluteAngleLauncherEncoder.getAbsolutePosition() * 360;
  }

  public double getLauncherConchAngleDegrees() {
    return (angleLauncherEncoder.getPosition() / Constants.Launcher.angleMotorGearboxReduction * 360.) % 360.;
  }

  public void resetToAbsolute() {
    double absolutePositionDegrees = (getAbsoluteLauncherAngleDegrees() - Constants.Launcher.angleOffset) % 360.;
    angleLauncherEncoder.setPosition(absolutePositionDegrees / 360. * Constants.Launcher.angleMotorGearboxReduction);
  }

  /**
   * The current target launcher angle.
   */
  double launcherAngle = 45;
  public double launcherSpeed = 1200;
  /**
   * Calculates the required conch angle from the wanted launcher angle.
   * @param angle
   */
  public void setLauncherAngle(Rotation2d angle) {
    launcherAngle = angle.getDegrees();
    SmartDashboard.putNumber("LauncherAngle", launcherAngle);
    
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
   * Gets the current target launcher angle.
   * @return
   */
  public double getLauncherAngle() {
    return launcherAngle;
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
   * Changes the current launcher speed by amount (launcher speed = launcher speed + amount).
   * @param amount
   */
  public void changeLauncherSpeed(double amount) {
    launcherSpeed += amount;
  }

  /**
   * 
   */
  public void setLauncherInverted(boolean inverted) {
    intakingNote = inverted;
  }

  public void runLauncher() {
    if (intakingNote) {
      topLaunchRollerPIDController.setReference(-1500, CANSparkMax.ControlType.kVelocity);
      bottomLaunchRollerPIDController.setReference(-1500, CANSparkMax.ControlType.kVelocity);    
    } else {
      topLaunchRollerPIDController.setReference(launcherSpeed, CANSparkMax.ControlType.kVelocity);
      bottomLaunchRollerPIDController.setReference(launcherSpeed, CANSparkMax.ControlType.kVelocity);    
    }
  }

  @Override
  public void periodic() {
    runLauncher();
    
    SmartDashboard.putNumber("Launcher absolute encoder", getAbsoluteLauncherAngleDegrees());
    SmartDashboard.putNumber("Launcher relative encoder", getLauncherConchAngleDegrees());
  }
}
