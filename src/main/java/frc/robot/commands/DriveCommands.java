package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Swerve swerve,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fieldRelativeSupplier) {
    return new InstantCommand(
      () -> {
        // We convert the joystick inputs to linear and angular velocities so we can separately square the magnitude and angle.
        // This allows for finer-grained control of the robot's motion.

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), Constants.Swerve.stickDeadband);
        Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Swerve.stickDeadband);

        // Square values while preserving sign on omega
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);

        // Calcaulate new linear velocity
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

        if(fieldRelativeSupplier.getAsBoolean()) {
          // Drive relative to the field

          // Convert to field relative speeds and send command

          // We use "Always blue origin" coordinates. For more information about why we flip for red, see the following documentation:
          // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin
          boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
          swerve.driveVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
              linearVelocity.getX() * Constants.Swerve.maxSpeed,
              linearVelocity.getY() * Constants.Swerve.maxSpeed,
              omega * Constants.Swerve.maxAngularVelocity,
              isFlipped
                ? swerve.getRotation().plus(new Rotation2d(Math.PI))
                : swerve.getRotation())
          );
        } else {
          // Drive relative to the robot

          // Convert to robot relative speeds and send command
          swerve.driveVelocity(
            new ChassisSpeeds(
              linearVelocity.getX() * Constants.Swerve.maxSpeed,
              linearVelocity.getY() * Constants.Swerve.maxSpeed,
              omega * Constants.Swerve.maxAngularVelocity
            )
          );
        }
      },
      swerve);
  }
}