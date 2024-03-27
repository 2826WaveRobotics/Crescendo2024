package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxConfig;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public final class Constants {
  /**
   * The current mode of the robot.  
   * This _must_ be changed before running simulations or on the real robot.  
   * There may be a better way to do this, but we need to differentiate between replays and normal simulations which doesn't seem possible.
   */
  public static final Mode currentMode = Mode.REAL;
  /** A situation that the robot is currently running in: real, simulation, or replay. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  /**
   * The deadband used for controller triggers.
   */
  public static final double triggerDeadband = 0.03;

  public static final boolean enableNonEssentialShuffleboard = true;

  public static final double fieldLengthMeters = Units.feetToMeters(54. + 1./12);

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 10;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20);
    // distance between centers of right and left wheels on robot
    public static final double wheelBase = Units.inchesToMeters(21);

    /** The swerve drive wheel diameters, in meters. */
    public static final double wheelDiameter = Units.inchesToMeters(4.0); //4

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final String[] moduleNames = {"frontLeft", "frontRight", "backLeft", "backRight"};

    /** The constraints to use while pathfinding. This doesn't apply to the paths followed at the end. */
    public static final PathConstraints pathfindingConstraints = new PathConstraints(3.4, 6.0, Units.degreesToRadians(540), Units.degreesToRadians(720));;
    public static final double trackingAngleControllerP = 7.5;
    public static final double trackingAngleControllerI = 0.0;
    public static final double trackingAngleControllerD = 0.0;

    // The locations of the modules on the robot, in meters.
    public static final Translation2d[] modulePositions = {
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),   // fl
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),  // fr
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // bl
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0)   // br
    };

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /**
     * The maximum robot movement speed in meters per second.
     */
    public static final double maxSpeed = 5.2;
    /**
     * The maximum robot angular velocity in radians per second.
     */
    public static final double maxAngularVelocity = 11.5;

    /**
     * The spark max config for the drive motors.
     */
    public static final CANSparkMaxConfig driveConfig = new CANSparkMaxConfig(
      CANSparkMax.IdleMode.kBrake,
      40, 50,
      100,
      12.0,
      Usage.kAll
    ).configurePIDSlot(0, 1e-4, 0.0, 0.0, 1. / 6784 /* Free speed of a NEO Vortex */);
    
    /**
     * The spark max config for the angle motors.
     */
    public static final CANSparkMaxConfig angleConfig = new CANSparkMaxConfig(
      CANSparkMax.IdleMode.kBrake,
      20, 30,
      100,
      12.0,
      Usage.kPositionOnly
    ).configurePIDSlot(0, 0.5, 5e-7, 0.0, 1. / 5700 /* Free speed of a NEO 1650 */);

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(349.013);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(119.355);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(66.884);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 41;
      public static final int angleMotorID = 42;
      public static final int canCoderID = 43;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(306.298);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class Auto {
    public static final double kMaxSpeedMetersPerSecond = 3; //3.25
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class Launcher {
    public static final int absoluteEncoderDIOPort = 0;

    public static final int topRollerCANID = 59;
    public static final int bottomRollerCANID = 52;
    public static final int angleMotorCANID = 27;

    public static final double wheelRadiusMeters = Units.inchesToMeters(2);

    public static CANSparkMaxConfig rollerConfig = new CANSparkMaxConfig(
      CANSparkMax.IdleMode.kCoast,
      30,
      40,
      100,
      12.0,
      CANSparkMaxUtil.Usage.kPositionOnly
    ).configurePIDSlot(0, 6e-5, 0.0, 0.0, 1 / 5700.);

    public static CANSparkMaxConfig angleConfig = new CANSparkMaxConfig(
      CANSparkMax.IdleMode.kBrake,
      10,
      15,
      100,
      12.0,
      CANSparkMaxUtil.Usage.kPositionOnly
    ).configurePIDSlot(0, 0.1, 0.0, 0.0, 0.0);
    
    /**
     * The idle launch roller velocity in revolutions per minute.
     */
    public static final double launchRollerVelocity = 120.; // 1500.0;
    /**
     * The launch roller velocity, when shooting, in revolutions per minute.
     */
    public static final double maxRollerVelocity = 3700.0;

    public static final Rotation2d softStopMarginLow = Rotation2d.fromDegrees(3);
    public static final Rotation2d softStopMarginHigh = Rotation2d.fromDegrees(40);

    public static final double angleMotorGearboxReduction = Math.pow(5.23, 3);

    /**
     * The conch angle offset. 0 degrees should be where the axle sits at the lowest point.
     */
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(100);

    /**
     * If we should invert the angle motor direction.
     */
    public static final boolean invertAngle = false;
  }

  public static final class NoteSensors {
    /**
     * The DIO port of the through beam sensor detecting if notes are in the intake.
     */
    public static final int intakeSensorDIOPort = 6;
    /**
     * The DIO port of the through beam sensor detecting if the note is in position.
     */
    public static final int noteInPositionSensorDIOPort = 2;
    /**
     * The DIO port of the through beam sensor detecting if the note is transitioning to the resting position.
     */
    public static final int noteInTransitionSensorDIOPort = 4;
  }

  public static final class Transport {
    public static final int topTransportMotorCANID = 45;
    public static final int bottomTransportMotorCANID = 54;

    public static final double rollerDiameterMeters = Units.inchesToMeters(1.525);

    public static final double transportGearRatio = 2.89;

    public static final CANSparkMaxConfig transportMotorConfig = new CANSparkMaxConfig(
      IdleMode.kCoast,
      20, 25,
      0.0, // We manually use a slew rate limiter so we can stop instantly.
      12.0,
      CANSparkMaxUtil.Usage.kPositionOnly
    ).configurePIDSlot(0, 1e-4, 0.0, 0.0, 1. / 5700.);

    /**
     * The speed that notes are moved into the launcher to shoot, in meters per second.
     */
    public static final double launchNoteTransportSpeed = 1.7;
    /**
     * The speed to move notes when ejecting.  
     * Notes are ejected when we have more than 1 note in the transport at a time.
     */
    public static final double ejectNoteSpeed = 0.5;
    /**
     * The intake speed (meaning the speed at the edge of the wheel/the speed that the belt moves at), in meters per second.
     */
    public static final double intakeSpeed = 5.0;
  }

  public static final class Climber {
    public static final int leftClimberMotorCANID = 55;
    public static final int rightClimberMotorCANID = 56;

    public static final int climbingSmartCurrentLimit = 30;
    public static final int climbingSecondaryCurrentLimit = 35;
    public static final int resetSmartCurrentLimit = 2;
    public static final int resetSecondaryCurrentLimit = 2;

    /**
     * The speed to run the climber motors at when climbing.
     */
    public static final double climberMotorSpeed = 4000;

    public static final CANSparkMaxConfig motorConfig = new CANSparkMaxConfig(
      IdleMode.kBrake,
      resetSmartCurrentLimit, resetSecondaryCurrentLimit,
      (1 / 0.2) * (5676 / climberMotorSpeed), // 0.2 second acceleration to the climbing speed
      12.0,
      Usage.kAll
    ).configurePIDSlot(0, 0.1, 0.0, 0.0, 0.0)         // Position controller
     .configurePIDSlot(1, 1e-7, 0.0, 0.0, 1. / 5600.); // Velocity controller

    public static final double fullUpRotations = 100;
  }
}
