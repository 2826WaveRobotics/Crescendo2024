package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public Rotation2d absoluteElevatorAngle = new Rotation2d();
    public double extensionHeight = 0;

    public double extensionOutputCurrent = 0;
    public double angleOutputCurrent = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Resets the encoder value for the extension motor to 0. */
  public default void resetExtensionEncoder() {}
  
  /** Resets the encoder value for the angle motor to 0. */
  public default void resetAngleEncoder() {}

  /**
   * Sets the extension motor speed in RPM. Positive numbers move the elevator downward.  
   * @param speed The target speed, in RPM.
   */
  public default void setExtensionSpeed(double speed) {}
  /**
   * Sets the angle motor speed in RPM. Positive numbers rotate toward the intake side of the robot, which is the direction of the elevator pointing upward.
   * @param speed
   */
  public default void setAngleSpeed(double speed) {}
  /**
   * Sets the angle motor position, where 0 is the resting position and positive numbers rotate upward.
   * @param angle
   */
  public default void setAnglePosition(Rotation2d angle) {}
  /**
   * Sets the extension position in meters, where 0 is fully contracted.
   * @param position
   */
  public default void setExtensionPosition(double position) {}
}
