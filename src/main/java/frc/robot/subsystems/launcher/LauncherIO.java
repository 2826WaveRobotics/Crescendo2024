package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    public Rotation2d absoluteLauncherAngle = new Rotation2d();
    public Rotation2d launcherRelativeConchAngle = new Rotation2d();
  }

  /** Runs the launch rollers at the specified speed in RPM. */
  public default void runRollers(double speed) {}

  /** Sets the angle of the conch motor in rotations. */
  public default void setAngleReference(double rotations) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LauncherIOInputs inputs) {}
}