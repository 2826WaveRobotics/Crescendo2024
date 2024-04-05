package frc.robot.controls;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.drive.FieldRelativeAcceleration;
import frc.lib.drive.FieldRelativeVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.Limelight;

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
        CenterNote
    }

    private AlignmentMode alignmentMode = AlignmentMode.Manual;

    private PIDController thetaController = new PIDController(Constants.Swerve.trackingAngleControllerP, Constants.Swerve.trackingAngleControllerI, Constants.Swerve.trackingAngleControllerD);

    private SwerveAlignmentController() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(7.5));
    }

    public void setAlignmentMode(AlignmentMode mode) {
        Logger.recordOutput("SwerveAlignmentController/AlignmentMode", mode.toString());

        alignmentMode = mode;
    }

    private boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public AlignmentMode getAlignmentMode() {
        return alignmentMode;
    }

    private LinearFilter velocityXFilter = LinearFilter.singlePoleIIR(0.05, 0.02);
    private LinearFilter velocityYFilter = LinearFilter.singlePoleIIR(0.05, 0.02);
    private LinearFilter accelerationXFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    private LinearFilter accelerationYFilter = LinearFilter.singlePoleIIR(0.2, 0.02);

    public double updateAllianceSpeakerDistance() {
        Swerve swerve = Swerve.getInstance();

        Translation2d currentPosition = swerve.getPose().getTranslation();
        FieldRelativeVelocity currentVelocity = swerve.getFieldRelativeVelocity();
        FieldRelativeAcceleration currentAcceleration = swerve.getFieldRelativeAcceleration();
        
        double speakerInward = -0.1;
        double speakerY = 5.55;
        double obtuseShiftY = 5.98 - speakerY;

        boolean isBlueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        Translation2d centerTargetLocation = isBlueAlliance ? new Translation2d(speakerInward, speakerY) : new Translation2d(Constants.fieldLengthMeters - speakerInward, speakerY);

        Rotation2d angleToTarget = centerTargetLocation.minus(currentPosition).getAngle().minus(Rotation2d.fromDegrees(
            isBlueAlliance ? 180 : 0
        ));
        speakerAngle = new Rotation2d(Math.abs(angleToTarget.getRadians()));

        double filteredVelocityX = velocityXFilter.calculate(currentVelocity.vx);
        double filteredVelocityY = velocityYFilter.calculate(currentVelocity.vy);
        double filteredAccelerationX = accelerationXFilter.calculate(currentAcceleration.ax);
        double filteredAccelerationY = accelerationYFilter.calculate(currentAcceleration.ay);

        Logger.recordOutput("SwerveAlignmentController/CurrentPosition", currentPosition);
        Logger.recordOutput("SwerveAlignmentController/CurrentVelocity/Mag", currentVelocity.getNorm());
        Logger.recordOutput("SwerveAlignmentController/CurrentAcceleration/Mag", currentAcceleration.getNorm());

        Translation2d targetLocation = centerTargetLocation;
        double startAngle = farInterpolationStartAngle.getDegrees();
        double angleWidth = farInterpolationWidth.getDegrees();
        if(speakerAngle.getDegrees() > startAngle) {
            double shiftY = (angleToTarget.getRadians() < 0 || angleToTarget.getRadians() > 180) ? -obtuseShiftY : obtuseShiftY;
            shiftY = isBlueAlliance ? -shiftY : shiftY;

            Translation2d movedTargetLocation = targetLocation.plus(new Translation2d(0, shiftY));
            if(speakerAngle.getDegrees() < startAngle + angleWidth) {
                double interpolation = (speakerAngle.getDegrees() - startAngle) / angleWidth;
                targetLocation = targetLocation.interpolate(movedTargetLocation, interpolation);
            } else {
                targetLocation = movedTargetLocation;
            }
        }
        Translation2d relativeTargetLocation = targetLocation.minus(currentPosition);
        double distance = relativeTargetLocation.getNorm();

        Logger.recordOutput("SwerveAlignmentController/RealDistance", distance);
        Logger.recordOutput("SwerveAlignmentController/TargetLocation", targetLocation);

        double shotTime = AutomaticLauncherControl.getShotTime(distance);

        double correctionFactor = 0.7; // Ideally would be close to 1, but just to compensate for other inaccuracies.

        // Poor man's Newton's method
        int iterations = 5; // Maximum number of iterations
        Translation2d correctedTargetPosition = targetLocation;
        for(int i = 0; i < iterations; i++) {
            double accelerationCompensationFactor = 0.1; // Roughly the pipeline latency in seconds
            correctedTargetPosition = new Translation2d(
                // Note that this doesn't use correctedTargetPosition as a base -- it uses targetLocation
                targetLocation.getX() - shotTime * (filteredVelocityX + filteredAccelerationX * accelerationCompensationFactor) * correctionFactor,
                targetLocation.getY() - shotTime * (filteredVelocityY + filteredAccelerationY * accelerationCompensationFactor) * correctionFactor
            );

            Translation2d relativeCorrectedTargetPosition = correctedTargetPosition.minus(currentPosition);
            distance = relativeCorrectedTargetPosition.getNorm();

            double oldShotTime = shotTime;
            shotTime = AutomaticLauncherControl.getShotTime(distance);

            if(Math.abs(shotTime - oldShotTime) < 0.01) {
                // Once we're close enough, stop
                break;
            }
        }

        allianceSpeakerDistance = distance;

        // Aim at correctedTargetPosition
        Translation2d currentTranslation = swerve.getPose().getTranslation();
        Translation2d relativeCorrectedTargetPosition = correctedTargetPosition.minus(currentTranslation);
        double angle = Math.atan2(relativeCorrectedTargetPosition.getY(), relativeCorrectedTargetPosition.getX());

        return angle;
    }
    
    public static Rotation2d farInterpolationStartAngle = Rotation2d.fromDegrees(35);
    public static Rotation2d farInterpolationWidth = Rotation2d.fromDegrees(15);
    public double allianceSpeakerDistance = 0.0;
    public Rotation2d speakerAngle = new Rotation2d();
    public boolean atTarget = false;

    private Rotation2d getTargetAngle() {
        switch (alignmentMode) {
            case AllianceSpeaker:
                // Add 180 degrees because we want to face the launcher toward the alliance speaker instead of the intake, which is the real front of the robot
                return Rotation2d.fromRadians(updateAllianceSpeakerDistance()).plus(Rotation2d.fromRadians(Math.PI));
            case Right:
                return Rotation2d.fromDegrees(isBlueAlliance() ? 270.0 : 90.0);
            case Backward:
                return Rotation2d.fromDegrees(isBlueAlliance() ? 180.0 : 0.0);
            case Left:
                return Rotation2d.fromDegrees(isBlueAlliance() ? 90.0 : 270.0);
            case CenterNote:
                return Swerve.getInstance().getPose().getRotation().plus(Limelight.getInstance().getIntakeNoteX());
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

        Logger.recordOutput("SwerveAlignmentController/TargetAngle", targetAngle.getDegrees());

        atTarget = thetaController.atSetpoint();

        return new ChassisSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            newOmegaRadiansPerSecond
        );
    }
}
