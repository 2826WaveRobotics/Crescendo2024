package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

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

  private final XboxController launcherJS;
  private double launchVelocityCmd;
  
  public Launcher(XboxController operatorCntr) {
    // Instantiate member variables and necessary code
    topRollerMotor = new CANSparkMax(Constants.Launcher.topRollerCANID, CANSparkMax.MotorType.kBrushless);
    bottomRollerMotor = new CANSparkMax(Constants.Launcher.bottomRollerCANID, CANSparkMax.MotorType.kBrushless);
    angleLauncherMotor = new CANSparkMax(Constants.Launcher.angleMotorCANID, CANSparkMax.MotorType.kBrushless);

    launcherJS = operatorCntr;

    angleLauncherEncoder = angleLauncherMotor.getEncoder();
    angleLauncherEncoder.setPositionConversionFactor(360 / Constants.Launcher.angleMotorGearboxReduction);
    absoluteAngleLauncherEncoder = new DutyCycleEncoder(0);
    absoluteAngleLauncherEncoder.setDutyCycleRange(0, 360); // convert position 0 - 1 to 0 - 360 (simulating degrees)

    topLaunchRollerPIDController = topRollerMotor.getPIDController();
    bottomLaunchRollerPIDController = bottomRollerMotor.getPIDController();

    anglePIDController = angleLauncherMotor.getPIDController();
    
    configMotorControllers();
    
    resetToAbsolute();
  }

  public void initLauncher() {
  }

  public double getAbsoluteLauncherAngleDegrees() {
    return absoluteAngleLauncherEncoder.getAbsolutePosition();
  }

  public void resetToAbsolute() {
    double absolutePositionDegrees = getAbsoluteLauncherAngleDegrees() - Constants.Launcher.angleOffset;
    angleLauncherEncoder.setPosition(absolutePositionDegrees * Constants.Launcher.angleMotorGearboxReduction);
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
  }

  /**
   * Calculates the required conch angle from the wanted launcher angle.
   * @param angle
   */
  public void setLauncherAngle(Rotation2d angle) {
    // All units here are in inches
    double conchToPivotDistance = 5.153673;
    double requiredRadius = Math.tan(angle.getRadians()) * conchToPivotDistance;
    
    double lowRadius = 0.1875; // in, from CAD
    double radiusIncrease = 3.4375; // in, from CAD: (4 -0.375/2-0.5/2)-(1/8)

    double conchAngleRadians = (requiredRadius - lowRadius) / radiusIncrease * (Math.PI * 2);
    if(conchAngleRadians < 0.0) {
      // The angle is lower than we can achieve
      conchAngleRadians = 0.0;
    }
    if(conchAngleRadians > Math.PI * 2) {
      // The angle is higher than we can achieve
      conchAngleRadians = Math.PI * 2;
    }
    anglePIDController.setReference(conchAngleRadians, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Enables the launch rollers.
   */
  public void launchRollersOn() {
    launchVelocityCmd = Constants.Launcher.launchRollerVelocity;

    if (Constants.Launcher.maxRollerVelocity < launchVelocityCmd) {
      launchVelocityCmd = Constants.Launcher.maxRollerVelocity;      
    }
  }

  /**
   * Disables the launch rollers.
   */
  public void launchRollersOff() {
    // Need to slew down to stop
    if (launchVelocityCmd > 50) {
      launchVelocityCmd = launchVelocityCmd - 50;
    }
    else {
      launchVelocityCmd = 0;
    }
  }

  @Override
  public void periodic() {
    // Insert periodic code here for testing   
    topLaunchRollerPIDController.setReference(launchVelocityCmd, CANSparkMax.ControlType.kVelocity);
    bottomLaunchRollerPIDController.setReference(launchVelocityCmd, CANSparkMax.ControlType.kVelocity);    
  }
}
