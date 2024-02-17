package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 10;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20);
    // distance between centers of right and left wheels on robot
    public static final double wheelBase = Units.inchesToMeters(21);
    public static final double wheelDiameter = Units.inchesToMeters(4.0); //4
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static String[] moduleNames ={"FRONT_LEFT","FRONT_RIGHT","BACK_LEFT","BACK_RIGHT"};

    // swerve drive kinematics object created based on module location
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),     // fl
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),    // fr
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),   // bl
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));   // br

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.016;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.15;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    /**
     * The maximum robot movement speed in meters per second.
     */
    public static final double maxSpeed = 4.5;
    /**
     * The maximum robot angular velocity in radians per second.
     */
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final CANSparkMax.IdleMode angleNeutralMode = CANSparkMax.IdleMode.kBrake;
    public static final CANSparkMax.IdleMode driveNeutralMode = CANSparkMax.IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(187);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(35);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 23;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(303.5);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(103.1);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(166.9);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 41;
      public static final int angleMotorID = 42;
      public static final int canCoderID = 43;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(307.35);
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
    public static final int topRollerCANID = 59;
    public static final int bottomRollerCANID = 52;
    public static final int angleMotorCANID = 27;
    public static final int angleMotorID = 0;

    public static final IdleMode rollerIdleMode = CANSparkMax.IdleMode.kCoast;
    /**
     * The current limit for the launch rollers, in amps, per motor.
     */
    public static final int rollerCurrentLimit = 30;
    
    /* launcher Roller motor PID Values */
    public static final double rollerKP = 6e-5;
    public static final double rollerKI = 0.0;
    public static final double rollerKD = 0.0;
    public static final double rollerKFF = 0.000175; 

    public static final IdleMode angleIdleMode = CANSparkMax.IdleMode.kBrake;
    /**
     * The current limit for the angle motor, in amps.
     */
    public static final int angleCurrentLimit = 10;
    
    /* launcher ANGLE motor PID Values */
    public static final double angleKP = 0.1;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.000175; 
    
    /* Launcher Voltage Compensation */
    public static final double voltageComp = 12.0;

    /**
     * The idle launch roller velocity in revolutions per minute.
     */
    public static final double launchRollerVelocity = 1500.0;
    /**
     * The launch roller velocity, when shooting, in revolutions per minute.
     */
    // public static final double maxRollerVelocity = 4340.0;
    public static final double maxRollerVelocity = 1700.0;

    public static final Rotation2d softStopMarginLow = Rotation2d.fromDegrees(10);
    public static final Rotation2d softStopMarginHigh = Rotation2d.fromDegrees(45);

    /**
     * The maximum rate of change of the launch roller velocity, in RPM per second.
     */
    public static final double launchVelocityRateLimit = 50000.;

    public static final double angleMotorGearboxReduction = Math.pow(5.23, 3);

    /**
     * The conch angle offset, in degrees. 0 degrees should be where the axle sits at the lowest point.
     */
    public static final double angleOffset = 108;

    /**
     * If we should invert the angle motor direction.
     */
    public static final boolean invertAngle = false;
  }

  public static final class Intake {
    public static final int frontIntakeMotorCANID = 46;
    // public static final int backIntakeMotorCANID = 51; // FIXME - Only 2 motors
    public static final int beltIntakeMotorCANID = 4;
      
    public static final double voltageComp = 12.0;

    /* Intake motors' PID Values */
    public static final double intakeKP = 6e-5;
    public static final double intakeKI = 0.0;
    public static final double intakeKD = 0.0;
    public static final double intakeKFF = 0.000175;

    public static final IdleMode intakeIdleMode = IdleMode.kCoast;

    /**
     * The current limit for each intake motor.
     */
    public static final int intakeCurrentLimit = 20;

    /**
     * The belt pulley radius, in meters.
     */
    public static final double beltPulleyRadius = Units.inchesToMeters(15.0 / 16.0);

    /**
     * The intake speed (meaning the speed at the edge of the wheel/the speed that the belt moves at), in meters per second.
     */
    public static final double intakeSpeed = 0.25;

    /**
     * The intake speed (meaning the speed at the edge of the wheel/the speed that the belt moves at)
     * when ejecting notes. This happens when we accidentally intake two notes. When this happens,
     * the front and back intake rollers run quickly in reverse to eject the note.
     */
    public static final double ejectSpeedMetersPerSecond = 0.5;

    /**
     * The deadband on the intake trigger.
     */
    public static final double intakeDeadband = 0.03;
  }

  public static final class Elevator {
    /**
     * The DIO port of the through beam sensor detecting if notes are in the intake.
     */
    public static final int intakeSensorDIOPort = 2;
    /**
     * The DIO port of the through beam sensor detecting if the note is in position.
     */
    public static final int noteInPositionSensorDIOPort = 4;
    /**
     * The DIO port of the through beam sensor detecting if the note is transitioning to the resting position.
     */
    public static final int noteInTransitionSensorDIOPort = 6;

    public static final int positionMotorCANID = 10;
    public static final int angleMotorCANID = 11;

    public static final IdleMode elevatorIdleMode = CANSparkMax.IdleMode.kCoast;
    /**
     * The current limit for the elevator position motor, in amps, per motor.
     */
    public static final int extRetCurrentLimit = 30;
    
    /* Elevator position motor PID Values */
    public static final double elevatorKP = 6e-5;
    public static final double elevatorKI = 0.0;
    public static final double elevatorKD = 0.0;
    public static final double elevatorKFF = 0.000175; 

    public static final IdleMode eAngleIdleMode = CANSparkMax.IdleMode.kBrake;
    /**
     * The current limit for the elevator angle motor, in amps.
     */
    public static final int eAngleCurrentLimit = 10;
    
    /* Elevator ANGLE motor PID Values */
    public static final double eAngleKP = 0.1;
    public static final double eAngleKI = 0.0;
    public static final double eAngleKD = 0.0;
    public static final double eAngleKFF = 0.000175; 
    
    /* Elevator Voltage Compensation */
    public static final double voltageComp = 12.0;
  
    /**
     * If we should invert the angle motor direction.
     */
    public static final boolean invertAngle = false;
    /**
     * If we should invert the position motor direction.
     */
    public static final boolean invertPosition = false;

    /**
     * The maximum velocity of the elevator, meters per second.
     */
    public static final double elevatorVelocity = 10;
    
    /**
     * The number of rotations on the elevator extension motor required to fully extend the elevator. 
     */
    public static final double rotationsForFullExtension = 65.57;
    /**
     * The total extension height of the elevator, in meters.
     */
    public static final double totalElevatorExtensionHeight = 1.016;

    /**
     * The target height of the elevator when it's fully extended.
     */
    public static final double elevatorExtendedHeight = 0.9;

    /**
     * The angle of the elevator when it's tilted upward.
     */
    public static final double elevatorUpAngle = 90;

    /**
     * The speed that the elevator angle motor should run at when changing angles.
     */
    public static final double angleSpeed = 10;

    /**
     * The DIO port of the elevator angle through-bore absolute encoder.
     */
    public static final int elevatorAbsoluteEncoderDIOPort = 8;
    /**
     * The angle offset of the elevator absolute encoder.
     */
    public static final double elevatorAbsoluteEncoderOffset = 0;
    /**
     * If we should invert the elevator absolute encoder.
     */
    public static final boolean invertAngleAbsoluteEncoder = false;
  }
  
  public static final class Transport {
    public static final int upperTransportMotorCANID = 57;
    public static final int lowerTransportMotorCANID = 54;

    public static final IdleMode transportIdleMode = CANSparkMax.IdleMode.kCoast;
    /**
     * The current limit for the launch rollers, in amps, per motor.
     */
    public static final int transportCurrentLimit = 5;
    public static final double voltageComp = 12;
    
    /* launcher Roller motor PID Values */
    public static final double transportKP = 6e-5;
    public static final double transportKI = 0.0;
    public static final double transportKD = 0.0;
    public static final double transportKFF = 0.000175;

    /**
     * The speed that notes are moved into the launcher to shoot, in meters per second.
     */
    public static final double launchNoteTransportSpeed = 5;
    /**
     * The speed that notes are moved out of the transport for putting them into the trap, in meters per second.
     */
    public static final double trapEjectSpeed = 1;
  }
}
