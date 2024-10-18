package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.lib.config.SwerveModuleConstants;

public class EncoderIOAnalogEncoder implements EncoderIO {
    private final Rotation2d absoluteEncoderOffset;
    private final AnalogEncoder encoder;

    public EncoderIOAnalogEncoder(SwerveModuleConstants moduleConstants) {
        encoder = new AnalogEncoder(new AnalogInput(moduleConstants.encoderInputID));
        absoluteEncoderOffset = moduleConstants.angleOffset;
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.rawAbsolutePosition = Rotation2d.fromRotations(encoder.getAbsolutePosition());
        inputs.absoluteTurnPosition = inputs.rawAbsolutePosition.minus(absoluteEncoderOffset);
    }
}
