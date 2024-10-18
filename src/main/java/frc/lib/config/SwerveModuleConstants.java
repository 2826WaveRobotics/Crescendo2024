package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  /// The CANCoder CAN ID or analog encoder DIO port.
  /// Not used if using a module with an analog encoder connected to the Spark MAX.
  public final int encoderInputID;
  public final Rotation2d angleOffset;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param encoderInputID
   * @param angleOffset
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int encoderInputID, Rotation2d angleOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.encoderInputID = encoderInputID;
    this.angleOffset = angleOffset;
  }
}
