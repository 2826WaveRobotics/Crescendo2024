package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.LimelightHelpers;

public interface LimelightIO {
  @AutoLog
  public static class LimelightIOInputs {
    public LimelightHelpers.PoseEstimate poseEstimateData = null;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LimelightIOInputs inputs) {}
}