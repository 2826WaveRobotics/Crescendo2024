package frc.robot.subsystems.drive;

import java.util.OptionalDouble;
import java.util.Queue;

import com.revrobotics.REVLibError;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.config.SwerveModuleConstants;
import frc.robot.Constants;

/**
 * Physics sim implementation of module IO.
 * This isn't a real implementation, although it eventually probably should be.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  protected final FlywheelSim driveMotor;
  protected final FlywheelSim turnMotor;

  // private final Queue<Double> timestampQueue;
  // private final Queue<Double> drivePositionQueue;
  // private final Queue<Double> turnPositionQueue;
  
  public SwerveModuleIOSim(SwerveModuleConstants moduleConstants) {
    driveMotor = new FlywheelSim(DCMotor.getNeoVortex(1), Constants.Swerve.driveGearRatio, 0.025);
    turnMotor = new FlywheelSim(DCMotor.getNEO(1), Constants.Swerve.angleGearRatio, 0.004096955);

    // timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();

    // drivePositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(
    //   () -> {
    //     double value = driveEncoder.getPosition();
    //     if (driveMotor.getLastError() == REVLibError.kOk) {
    //       return OptionalDouble.of(value);
    //     } else {
    //       return OptionalDouble.empty();
    //     }
    //   }
    // );
    // turnPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(
    //   () -> {
    //     double value = turnRelativeEncoder.getPosition();
    //     if (driveMotor.getLastError() == REVLibError.kOk) {
    //       return OptionalDouble.of(value);
    //     } else {
    //       return OptionalDouble.empty();
    //     }
    //   }
    // );

  }
  
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()) / Constants.Swerve.driveGearRatio;
    // inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / Constants.Swerve.driveGearRatio;
    // inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

    // inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / Constants.Swerve.angleGearRatio);

    // inputs.turnReportedAbsolutePosition = inputs.turnPosition;
    // inputs.turnAbsolutePosition = inputs.turnPosition;
    
    // inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / Constants.Swerve.angleGearRatio;
    // inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

    // inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
    //   .mapToDouble((Double value) -> Units.rotationsToRadians(value) / Constants.Swerve.driveGearRatio)
    //   .toArray();
    // inputs.odometryTurnPositions = turnPositionQueue.stream()
    //   .map((Double value) -> Rotation2d.fromRotations(value / Constants.Swerve.angleGearRatio))
    //   .toArray(Rotation2d[]::new);
    
    // timestampQueue.clear();
    // drivePositionQueue.clear();
    // turnPositionQueue.clear();
  }

  @Override
  public void setDriveVelocity(double rpm) {

  }

  @Override
  public void setTurnAngle(Rotation2d angle) {

  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
  
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
  
  }

  @Override
  public void setCharacterizationDriveVoltage(double voltage) {
  
  }
}