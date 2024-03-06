package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// List class features here, including any motors, sensors, and functionality:
// 2 Launcher motors to score notes
// 1 motor to aim (~30-60 degree window)
// 1 CAN Coder for getting the absolute angle
public class Launcher extends SubsystemBase {
  private static Launcher instance = null;
  public static Launcher getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance = new Launcher(new LauncherIOReal());   
          return instance;
        default:
          instance = new Launcher(new LauncherIO() {});   
          return instance;
      }
    }
    return instance;
  }

  private LauncherIO launcherIO;
  private LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();
  
  private Launcher(LauncherIO launcherIO) {
    this.launcherIO = launcherIO;
    
    launchRollersSlow();
  }

  /**
   * The current target launcher angle.
   */
  public double launcherAngle = 45;
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
    double pivotToConchReactionBarDistance = 4.507;
    double angleOffsetRadians = Units.degreesToRadians(23.53);
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

    launcherIO.setAngleReference(Rotation2d.fromRadians(conchAngleRadians).getRotations() * Constants.Launcher.angleMotorGearboxReduction);
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
   * Sets the launch roller speed.
   * @param speed
   */
  public void setLauncherSpeed(double speed) {
    launcherSpeed = speed;
  }

  @Override
  public void periodic() {
    launcherIO.updateInputs(inputs);
    launcherIO.runRollers(launcherSpeed);
    
    SmartDashboard.putNumber("LauncherSpeed", launcherSpeed);
    
    SmartDashboard.putNumber("Launcher absolute encoder", inputs.absoluteLauncherAngle.getDegrees());
    SmartDashboard.putNumber("Launcher relative encoder", inputs.launcherRelativeConchAngle.getDegrees());
  }
}
