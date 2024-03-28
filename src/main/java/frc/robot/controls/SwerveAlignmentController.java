package frc.robot.controls;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.drive.FieldRelativeAcceleration;
import frc.lib.drive.FieldRelativeVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.transport.Transport.TransportState;

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
        thetaController.setTolerance(Units.degreesToRadians(7.5));
    }

    public void setAlignmentMode(AlignmentMode mode) {
        alignmentMode = mode;
    }

    private boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public AlignmentMode getAlignmentMode() {
        return alignmentMode;
    }

    public double allianceSpeakerDistance = 0.0;
    public boolean atTarget = false;

    private Rotation2d getTargetAngle() {
        switch (alignmentMode) {
            case AllianceSpeaker:
                Swerve swerve = Swerve.getInstance();

                Translation2d currentPosition = swerve.getPose().getTranslation();
                FieldRelativeVelocity currentVelocity = swerve.getFieldRelativeVelocity();
                FieldRelativeAcceleration currentAcceleration = swerve.getFieldRelativeAcceleration();

                double speakerInward = -0.1;
                boolean isBlueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
                Translation2d targetLocation = isBlueAlliance ? new Translation2d(speakerInward, 5.55) : new Translation2d(Constants.fieldLengthMeters - speakerInward, 5.55);

                Translation2d relativeTargetLocation = targetLocation.minus(currentPosition);
                double distance = relativeTargetLocation.getNorm();

                double shotTime = AutomaticLauncherControl.getShotTime(distance);

                // Poor man's Newton's method
                int iterations = 10; // Maximum number of iterations
                Translation2d correctedTargetPosition = targetLocation;
                for(int i = 0; i < iterations; i++) {
                    double accelerationCompensationFactor = 0.1;
                    correctedTargetPosition = new Translation2d(
                        // Note that this doesn't use correctedTargetPosition as a base -- it uses targetLocation
                        targetLocation.getX() - shotTime * (currentVelocity.vx + currentAcceleration.ax * accelerationCompensationFactor),
                        targetLocation.getY() - shotTime * (currentVelocity.vy + currentAcceleration.ay * accelerationCompensationFactor)
                    );

                    Translation2d relativeCorrectedTargetPosition = correctedTargetPosition.minus(currentPosition);
                    distance = relativeCorrectedTargetPosition.getNorm();

                    double oldShotTime = shotTime;
                    shotTime = AutomaticLauncherControl.getShotTime(distance);

                    if(Math.abs(shotTime - oldShotTime) < 0.005) {
                        // Once we're close enough, stop
                        break;
                    }
                }

                allianceSpeakerDistance = distance;

                // Aim at correctedTargetPosition
                Translation2d currentTranslation = swerve.getPose().getTranslation();
                Translation2d relativeCorrectedTargetPosition = correctedTargetPosition.minus(currentTranslation);
                double angle = Math.atan2(relativeCorrectedTargetPosition.getY(), relativeCorrectedTargetPosition.getX());
                
                // Add 180 degrees because we want to face the launcher toward the alliance speaker instead of the intake, which is the real front of the robot
                return Rotation2d.fromRadians(angle).plus(Rotation2d.fromRadians(Math.PI));
            case Right:
                return Rotation2d.fromDegrees(isBlueAlliance() ? 270.0 : 90.0);
            case Backward:
                return Rotation2d.fromDegrees(isBlueAlliance() ? 180.0 : 0.0);
            case Left:
                return Rotation2d.fromDegrees(isBlueAlliance() ? 90.0 : 270.0);
            default:
                return Rotation2d.fromDegrees(isBlueAlliance() ? 0.0 : 180.0);
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

        atTarget = thetaController.atSetpoint();

        return new ChassisSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            newOmegaRadiansPerSecond
        );
    }
}
