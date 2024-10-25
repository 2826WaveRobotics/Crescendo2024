package frc.robot.subsystems.drive;

import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.config.SwerveModuleConstants;

public class EncoderIOSparkMaxAnalogEncoder implements EncoderIO {
    private final Rotation2d absoluteEncoderOffset;
    private final SparkAbsoluteEncoder absoluteEncoder;

    public EncoderIOSparkMaxAnalogEncoder(SwerveModuleConstants moduleConstants, SparkAbsoluteEncoder encoder) {
        absoluteEncoderOffset = moduleConstants.angleOffset;
        absoluteEncoder = encoder;
        absoluteEncoder.setPositionConversionFactor(1); // Ensure the output is in rotations
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.rawAbsolutePosition = Rotation2d.fromRotations(absoluteEncoder.getVelocity());
        inputs.absoluteTurnPosition = inputs.rawAbsolutePosition.minus(absoluteEncoderOffset);
    }
}
