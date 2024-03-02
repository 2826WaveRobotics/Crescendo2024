package frc.robot.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
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
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.LimelightHelpers;
import frc.lib.util.LocalADStarAK;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private static Swerve instance = null;
  public static Swerve getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          return new Swerve(
            new GyroIOPigeon2(),
            new SwerveModuleIO[] {
              new SwerveModuleIOSparkMax(Constants.Swerve.Mod0.constants),
              new SwerveModuleIOSparkMax(Constants.Swerve.Mod1.constants),
              new SwerveModuleIOSparkMax(Constants.Swerve.Mod2.constants),
              new SwerveModuleIOSparkMax(Constants.Swerve.Mod3.constants)
            });
        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          return new Swerve(
            new GyroIO() {},
            new SwerveModuleIO[] { new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim() }
          );
        default:
          // Replayed robot, disable IO implementations
          return new Swerve(
            new GyroIO() {},
            new SwerveModuleIO[] { new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {} }
          );
      }
    }
    return instance;
  }

  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final SwerveDriveKinematics kinematics;
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
  
  /**
   * The system identification routine used to tune the swerve modules.
   */
  private final SysIdRoutine sysIdRoutine;

  
  private Rotation2d rawGyroRotation = new Rotation2d();
  /**
   * The last known module positions.  
   * Used for delta tracking.
   */
  private SwerveModulePosition[] lastModulePositions;

  private Swerve(
    GyroIO gyroIO,
    SwerveModuleIO[] moduleIOs
  ) {
    this.gyroIO = gyroIO;
    
    swerveModules = new SwerveModule[moduleIOs.length];
    for (int i = 0; i < moduleIOs.length; i++) {
      swerveModules[i] = new SwerveModule(i, moduleIOs[i]);
    }

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModulePositions[i] = swerveModules[i].getPosition();
    }
    
    lastModulePositions = swerveModulePositions;

    Pose2d startPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d());

    kinematics = new SwerveDriveKinematics(Constants.Swerve.modulePositions);

    swerveOdometry = new SwerveDrivePoseEstimator(
      kinematics,
      getYaw(),
      swerveModulePositions,
      startPose,
      stateStdDevs,
      visionMeasurementStdDevs
    );
    
    resetRotation();

    // utility used to build auto paths
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        // We don't need to pass PID constants, so we don't for now.
        // new PIDConstants(5, 0, 0), // Translation PID constants
        // new PIDConstants(5, 0, 0), // Rotation PID constants
        Constants.Swerve.maxSpeed, // Max module speed, in m/s
        Constants.Swerve.wheelBase, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
      ),
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red,
      this // Reference to this subsystem to set requirements
    );
    
    // We need to use a modified version of the default pathfinder to allow it to work with AdvantageKit.
    Pathfinding.setPathfinder(new LocalADStarAK());
    
    PathPlannerLogging.setLogActivePathCallback((activePath) -> {
      Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    });
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
      Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    });

    // Configure SysId
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
      new SysIdRoutine.Mechanism(
        voltage -> {
          for (SwerveModule mod : swerveModules) {
            mod.runCharacterization(voltage.in(BaseUnits.Voltage));
          }
        },
        null,
        this));

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
    return kinematics.toChassisSpeeds(getStates());
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

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      states[i] = swerveModules[i].getPosition();
    }
    return states;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void driveVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Swerve.maxSpeed);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < swerveModules.length; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = swerveModules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    driveVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < swerveModules.length; i++) {
      headings[i] = Constants.Swerve.modulePositions[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      states[i] = swerveModules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  /**
   * Returns the current odometry rotation.
   * NOTE: This isn't necessarily the same as the gyro angle.
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Resets the odometry to face the current direction.
   */
  public void resetRotation() {
    setPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    swerveOdometry.addVisionMeasurement(visionPose, timestamp);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleIndex] = mod.getState();
    }
    return states;
  }

  /**
   * Gets the current yaw of the gyro, relative to when we last zeroed it.  
   * Positive Z/yaw is left.
   * @return
   */
  public Rotation2d getYaw() {
    return gyroInputs.yawPosition;
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (SwerveModule module : swerveModules) {
      module.updateInputs();
    }
    odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (SwerveModule module : swerveModules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (SwerveModule module : swerveModules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry.
    // We use many samples per update to vastly increase the accuracy of the odometry.

    double[] sampleTimestamps = swerveModules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < swerveModules.length; moduleIndex++) {
        modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
          modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
          modulePositions[moduleIndex].angle
        );
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      swerveOdometry.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    field.setRobotPose(getPose());

    updateOdometryPose();
  }
}
