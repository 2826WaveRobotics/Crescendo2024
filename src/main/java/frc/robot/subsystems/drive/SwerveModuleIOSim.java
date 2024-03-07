package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.config.SwerveModuleConstants;
import frc.robot.Constants;

/**
 * Physics sim implementation of module IO.
 * This isn't a real implementation, although it eventually probably should be.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  protected final CANSparkMax driveSparkMax;
  protected final CANSparkMax turnSparkMax;

  private final SparkPIDController drivePIDController;
  private final SparkPIDController turnPIDController;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;

  public SwerveModuleIOSim(SwerveModuleConstants moduleConstants) {
    driveSparkMax = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    turnSparkMax = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    drivePIDController = driveSparkMax.getPIDController();
    turnPIDController = turnSparkMax.getPIDController();

    Constants.Swerve.driveConfig.configure(driveSparkMax, drivePIDController, false);
    Constants.Swerve.angleConfig.configure(turnSparkMax, turnPIDController, false);

    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(0.0);
    turnPIDController.setPositionPIDWrappingMaxInput(Constants.Swerve.angleGearRatio);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int)(1000.0 / SwerveModule.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int)(1000.0 / SwerveModule.ODOMETRY_FREQUENCY));

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();

    REVPhysicsSim.getInstance().addSparkMax(this.driveSparkMax, DCMotor.getNeoVortex(1));
    REVPhysicsSim.getInstance().addSparkMax(this.turnSparkMax, DCMotor.getNeoVortex(1));
  }
}