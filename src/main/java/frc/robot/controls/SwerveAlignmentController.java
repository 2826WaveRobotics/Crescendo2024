package frc.robot.controls;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;

/**
 * This class manages updating our input robot speeds to target the current goal.
 */
public class SwerveAlignmentController {
    private static SwerveAlignmentController instance = null;
    public static SwerveAlignmentController getInstance() {
        if (instance == null) {
            instance = new SwerveAlignmentController();
        }
        return instance;
    }
    
    public enum AlignmentMode {
        AllianceSpeaker,
        Forward,
        Right,
        Backward,
        Left,
        Manual,
    }

    private AlignmentMode alignmentMode = AlignmentMode.Manual;

    private PIDController thetaController = new PIDController(Constants.Swerve.trackingAngleControllerP, Constants.Swerve.trackingAngleControllerI, Constants.Swerve.trackingAngleControllerD);

    private SwerveAlignmentController() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setAlignmentMode(AlignmentMode mode) {
        alignmentMode = mode;
    }

    private Rotation2d getTargetAngle() {
        switch (alignmentMode) {
            case AllianceSpeaker:
                Swerve swerve = Swerve.getInstance();

                // Adjust for the robot's speed since by the time we reach the target, the robot will have moved
                double lookaheadDistance = swerve.getRobotSpeed() * 0.05; // 0.05 is abritrary; let's say it takes 50ms to rotate to the proper angle
                Pose2d pose = swerve.getPose().transformBy(new Transform2d(lookaheadDistance, 0.0, new Rotation2d()));
                Translation2d currentTranslation = pose.getTranslation();

                boolean isBlueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
                Translation2d allianceSpeakerTranslation = isBlueAlliance ? new Translation2d(0.2, 5.55) : new Translation2d(16.34, 5.55);

                return Rotation2d.fromRadians(
                    -Math.atan2(
                        allianceSpeakerTranslation.getY() - currentTranslation.getY(),
                        allianceSpeakerTranslation.getX() - currentTranslation.getX()
                    ) // Negative because we want to face the launcher toward the alliance speaker instead of the intake, which is the real front of the robot
                );
            case Forward:
                return Rotation2d.fromDegrees(0.0);
            case Right:
                return Rotation2d.fromDegrees(270.0);
            case Backward:
                return Rotation2d.fromDegrees(180.0);
            case Left:
                return Rotation2d.fromDegrees(90.0);
            default:
                return Rotation2d.fromDegrees(0.0);
        }
    }

    public void reset() {
        alignmentMode = AlignmentMode.Manual;
        thetaController.reset();
    }

    public ChassisSpeeds updateSpeedsToAlign(ChassisSpeeds speeds) {
        // If we are in manual mode, we don't want to change the speeds.
        if (alignmentMode == AlignmentMode.Manual) {
            return speeds;
        }

        // If we are in any other mode, we want to change the speeds to align with the target angle.
        Rotation2d targetAngle = getTargetAngle();

        Rotation2d currentAngle = Swerve.getInstance().getPose().getRotation();

        double newOmegaRadiansPerSecond = Math.min(
            thetaController.calculate(currentAngle.getRadians(), targetAngle.getRadians()),
            Constants.Swerve.maxAngularVelocity
        );

        return new ChassisSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            newOmegaRadiansPerSecond
        );
    }
}
