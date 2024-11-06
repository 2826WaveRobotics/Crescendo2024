package frc.robot.subsystems.drive;

import com.revrobotics.SparkAnalogSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.config.SwerveModuleConstants;

public class EncoderIOSparkMaxAnalogEncoder implements EncoderIO {
    private final Rotation2d absoluteEncoderOffset;
    private final SparkAnalogSensor absoluteEncoder;

    public EncoderIOSparkMaxAnalogEncoder(SwerveModuleConstants moduleConstants, SparkAnalogSensor encoder) {
        absoluteEncoderOffset = moduleConstants.angleOffset;
        absoluteEncoder = encoder;
        absoluteEncoder.setInverted(false);
        absoluteEncoder.setPositionConversionFactor(1/21.2); // ??????
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.rawAbsolutePosition = Rotation2d.fromRotations(absoluteEncoder.getPosition());
        inputs.absoluteTurnPosition = inputs.rawAbsolutePosition.minus(absoluteEncoderOffset);
    }
}
