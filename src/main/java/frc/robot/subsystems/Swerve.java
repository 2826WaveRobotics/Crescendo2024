//FIX ME --- MODULE POSITION!!!!!!!!!

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;

  // TESTING
  // previously swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), swerveModulePositions);
  private SwerveDrivePoseEstimator swerveOdometry;
  private SwerveModule[] swerveModules;

  private Field2d field;

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();
    
    swerveModules =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
      swerveModules[0].getPosition(),
      swerveModules[1].getPosition(),
      swerveModules[2].getPosition(),
      swerveModules[3].getPosition()
    };

    Pose2d startPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d());

    swerveOdometry = new SwerveDrivePoseEstimator(
      Constants.Swerve.swerveKinematics,
      getYaw(),
      swerveModulePositions,
      startPose,
      stateStdDevs,
      visionMeasurementStdDevs
    );

    // utility used to build auto paths
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
          // PIDConstants(5, 0, 0) is a sensible default; maybe we shouldn't be using the teleop values in autonomous
          new PIDConstants(5, 0, 0), // Translation PID constants
          new PIDConstants(5, 0, 0), // Rotation PID constants
          1, // Max module speed, in m/s
          0.4, // Drive base radius in meters. Distance from robot center to furthest module.
          new ReplanningConfig(true, false) // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
  }

  /**
   * Gets the absolute speed of the robot in meters per second.
   * @return
   */
  public double getRobotSpeed() {
    ChassisSpeeds chassisSpeeds = getRobotRelativeSpeeds();
    return Math.sqrt(chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public void updateOdometryPose() {
    boolean hasTargets = LimelightHelpers.getTV("limelight");
    if(!hasTargets) return;

    Pose2d pose = LimelightHelpers.getBotPose2d("limelight");
    Translation2d poseTranslation = pose.getTranslation();

    // Get the position of the primary tag. If it's further than 5 meters away, discard the data.
    double distance = LimelightHelpers.getTargetPose3d_CameraSpace("limelight").getTranslation().getDistance(new Translation3d());
    if(distance > 5) return;

    // Offset the pose to the center of the field because the limelight returns (0, 0)
    // as the center instead of (16.45, 8.09). This should probably be fixed in
    // LimelightHelpers instead, but this is easiest for now.
    Pose2d fixedPose = new Pose2d(new Translation2d(
      16.4592 / 2 + poseTranslation.getX(),
      8.09625 / 2 + poseTranslation.getY()
    ), pose.getRotation());

    double[] botpose = LimelightHelpers.getBotPose("limelight");
    if(botpose.length == 0) return;

    // Scale the vision measurement expected standard deviation exponentially by the distance 
    double standardDeviationScalar = Math.max(0, 0.2 * Math.pow(2.2, distance) - 0.03);
    // SmartDashboard.putNumber("Limelight closest tag distance", distance);
    // SmartDashboard.putNumber("Limelight standard deviation scalar", standardDeviationScalar);
    // SmartDashboard.putString("Vision measurement predicted standard deviation", 
    //   "(" + Math.round(visionMeasurementStdDevs.get(0, 0) * standardDeviationScalar * 1000) / 1000. + ", " +
    //   Math.round(visionMeasurementStdDevs.get(1, 0) * standardDeviationScalar * 1000) / 1000. + ", " +
    //   Math.round(visionMeasurementStdDevs.get(2, 0) * standardDeviationScalar * 1000) / 1000. + ")");
    addVisionMeasurement(fixedPose, botpose[6], standardDeviationScalar);
  }

  public void addVisionMeasurement(Pose2d pose, double pipelineLatency, double standardDeviationScalar) {
    swerveOdometry.addVisionMeasurement(
      pose,
      Timer.getFPGATimestamp() - (pipelineLatency / 1000.0),
      visionMeasurementStdDevs.times(standardDeviationScalar)
    );
  }

  /**
   * Updates the swerve drivetrain with the specified values.
   * @param velocity
   * @param rotation The angular velocity, in radians per second.
   * @param fieldRelative 
   * @param isOpenLoop 
   */
  public void drive(Translation2d velocity, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(
              velocity.getX(), velocity.getY(), rotation, getYaw()
            )
          : new ChassisSpeeds(velocity.getX(), velocity.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  /**
   * Resets the robot's position on the field.
   * @param pose
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), new SwerveModulePosition[] {
      swerveModules[0].getPosition(), 
      swerveModules[1].getPosition(), 
      swerveModules[2].getPosition(), 
      swerveModules[3].getPosition()
    }, pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * Resets the yaw value of the gyroscope to zero. 
   */
  public void zeroGyro() {
    gyro.setYaw(0);
  }

  /**
   * Gets the current yaw of the gyro, relative to when we last zeroed it.  
   * Positive Z/yaw is left.
   * @return
   */
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble())
        : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), new SwerveModulePosition[] {
      swerveModules[0].getPosition(), 
      swerveModules[1].getPosition(), 
      swerveModules[2].getPosition(), 
      swerveModules[3].getPosition()
    });

    field.setRobotPose(getPose());

    updateOdometryPose();

    for (SwerveModule mod : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCANcoderAbsoluteAngle().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
