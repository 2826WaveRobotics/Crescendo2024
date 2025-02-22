package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drive.FieldRelativeAcceleration;
import frc.lib.drive.FieldRelativeVelocity;
import frc.lib.util.LocalADStarAK;
import frc.lib.util.ShuffleboardContent;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.visualization.NoteVisualizer;

public class Swerve extends SubsystemBase {
  private static Swerve instance = null;
  public static Swerve getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance = new Swerve(
            new GyroIOPigeon2(),
            new SwerveModuleIO[] {
              new SwerveModuleIOSparkMax(Constants.Swerve.mod0Constants),
              new SwerveModuleIOSparkMax(Constants.Swerve.mod1Constants),
              new SwerveModuleIOSparkMax(Constants.Swerve.mod2Constants),
              new SwerveModuleIOSparkMax(Constants.Swerve.mod3Constants)
            }
          );
          return instance;
        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new Swerve(
            new GyroIO() {},
            new SwerveModuleIO[] { new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim() }
          );
          return instance;
        default:
          // Replayed robot, disable IO implementations
          instance = new Swerve(
            new GyroIO() {},
            new SwerveModuleIO[] { new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {} }
          );
          return instance;
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
  public void selectedAutoChanged(String auto) {
    try {
      Pose2d startPose = PathPlannerAuto.getStaringPoseFromAutoFile(auto);
      field.getObject("Auto start").setPose(startPose);

      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(auto);
      List<Pose2d> pathPoses = new ArrayList<>();
      for (PathPlannerPath path : paths) {
        for(Pose2d pose : path.getPathPoses()) {
          pathPoses.add(pose);
        }
      }
      field.getObject("Path poses").setPoses(pathPoses);
    } catch(RuntimeException e) {
      field.getObject("Auto start").setPoses(new Pose2d[0]);
      field.getObject("Path poses").setPoses(new Pose2d[0]);
    }
  }
  public void autoStart() {
    field.getObject("Auto start").setPoses(new Pose2d[0]);
    field.getObject("Path poses").setPoses(new Pose2d[0]);
  }

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
    
    SparkMaxOdometryThread.getInstance().start();

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
      Limelight.stateStdDevs,
      Limelight.visionMeasurementStdDevs
    );
    
    resetRotation();

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(7, 0, 0), // Translation PID constants
        new PIDConstants(6, 0, 0), // Rotation PID constants
        Constants.Swerve.maxVelocity, // Max module speed, in m/s
        Math.sqrt(Constants.Swerve.wheelBase * Constants.Swerve.wheelBase + Constants.Swerve.trackWidth * Constants.Swerve.trackWidth) / 2, // Drive base radius in meters. Distance from robot center to furthest module.
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
    ShuffleboardContent.competitionTab.add("Field", field)
      .withPosition(5, 0)
      .withSize(5, 4);

    if(Constants.enableNonEssentialShuffleboard) {
      Shuffleboard.getTab("Notes").addString("Odometry position", () -> ("(" + getPose().getX() + ", " + getPose().getY() + ")"));
      
      Shuffleboard.getTab("Notes").addNumber("Speaker distance", () -> {
        Swerve swerve = Swerve.getInstance();

        Translation2d currentPosition = swerve.getPose().getTranslation();
        
        double speakerInward = -0.1;
        double speakerY = 5.55;

        boolean isBlueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        Translation2d targetLocation = isBlueAlliance ? new Translation2d(speakerInward, speakerY) : new Translation2d(Constants.fieldLengthMeters - speakerInward, speakerY);

        Translation2d relativeTargetLocation = targetLocation.minus(currentPosition);
        double distance = relativeTargetLocation.getNorm();

        return distance;
      });
      Shuffleboard.getTab("Notes").addNumber("Lob distance", () -> {
        Swerve swerve = Swerve.getInstance();

        Translation2d currentPosition = swerve.getPose().getTranslation();

        double targetX = 1.38;
        double targetY = 7.06;

        boolean isBlueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        Translation2d targetLocation = isBlueAlliance ? new Translation2d(targetX, targetY) : new Translation2d(Constants.fieldLengthMeters - targetX, targetY);

        Translation2d relativeTargetLocation = targetLocation.minus(currentPosition);
        double distance = relativeTargetLocation.getNorm();

        return distance;
      });
    }

    NoteVisualizer.setRobotPoseSupplier(this::getPose);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
  }

  /** Resets the relative angle encoder to the CANCoder's absolute position on every module. */
  public void resetToAbsolute() {
    for(var mod : swerveModules) {
      mod.resetToAbsolute();
    }
  }

  private double robotSpeed = 0.0;
  private FieldRelativeVelocity velocity = new FieldRelativeVelocity();
  private FieldRelativeVelocity lastVelocity = new FieldRelativeVelocity();
  private FieldRelativeAcceleration acceleration = new FieldRelativeAcceleration();

  /**
   * Gets the absolute speed of the robot in meters per second.
   * @return
   */
  @AutoLogOutput(key = "Drive/Speed")
  public double getRobotSpeed() {
    return robotSpeed;
  }
  public FieldRelativeVelocity getFieldRelativeVelocity() {
    return velocity;
  }
  public FieldRelativeAcceleration getFieldRelativeAcceleration() {
    return acceleration;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, double standardDeviationScalar) {
    swerveOdometry.addVisionMeasurement(
      pose,
      timestampSeconds,
      Limelight.visionMeasurementStdDevs.times(standardDeviationScalar)
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
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Swerve.maxVelocity);

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

  public double[] getOffsets() {
    return new double[] {
      swerveModules[0].getReportedAbsoluteAngleDegrees(),
      swerveModules[1].getReportedAbsoluteAngleDegrees(),
      swerveModules[2].getReportedAbsoluteAngleDegrees(),
      swerveModules[3].getReportedAbsoluteAngleDegrees()
    };
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

  Tracer timeTracer = new Tracer();

  @Override
  public void periodic() {
    double startTime = Logger.getRealTimestamp();
    timeTracer.clearEpochs();

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    timeTracer.addEpoch("Gyro updates");
    for (SwerveModule module : swerveModules) {
      module.updateInputs();
    }
    timeTracer.addEpoch("Module input updates");
    odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (SwerveModule module : swerveModules) {
      module.periodic();
    }
    timeTracer.addEpoch("Module periodic");

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (SwerveModule module : swerveModules) {
        module.stop();
      }
      timeTracer.addEpoch("Module stop");
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
    
    timeTracer.addEpoch("Odometry updates");

    double executionTime = Logger.getRealTimestamp() - startTime;
    if(executionTime > 10000) { // More than 0.01 seconds
      System.out.println("Swerve time took over 0.01 seconds: " + ((Logger.getRealTimestamp() - startTime) / 1000000) + "s");
      timeTracer.printEpochs();
    }
    Logger.recordOutput("Drive/ExecutionTimeMS", executionTime * 0.001);

    field.setRobotPose(getPose());

    // Update speed, velocity, and accceleration
    
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), rawGyroRotation);
    robotSpeed = Math.sqrt(
      chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond +
      chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond
    );

    velocity = new FieldRelativeVelocity(chassisSpeeds, gyroInputs.yawPosition);
    acceleration = new FieldRelativeAcceleration(velocity, lastVelocity, 0.02);
    lastVelocity = velocity;
  }
}
