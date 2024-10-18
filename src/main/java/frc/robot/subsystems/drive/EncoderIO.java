package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {
  @AutoLog
  public static class EncoderIOInputs {
    public Rotation2d rawAbsolutePosition = new Rotation2d();
    public Rotation2d absoluteTurnPosition = new Rotation2d();
  }

  public default void updateInputs(EncoderIOInputs inputs) {}
}