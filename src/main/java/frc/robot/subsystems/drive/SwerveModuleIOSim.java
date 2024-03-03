package frc.robot.subsystems.drive;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.config.SwerveModuleConstants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class SwerveModuleIOSim extends SwerveModuleIOSparkMax {
  public SwerveModuleIOSim(SwerveModuleConstants constants) {
    super(constants);

    REVPhysicsSim.getInstance().addSparkMax(this.driveSparkMax, DCMotor.getNeoVortex(1));
    REVPhysicsSim.getInstance().addSparkMax(this.turnSparkMax, DCMotor.getNeoVortex(1));
  }
}