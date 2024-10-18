package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.config.SwerveModuleConstants;

public class EncoderIOSparkMaxAnalogEncoder implements EncoderIO {
    private final Rotation2d absoluteEncoderOffset;
    private final CANSparkMax sparkMax;
    private final SparkAbsoluteEncoder absoluteEncoder;

    public EncoderIOSparkMaxAnalogEncoder(SwerveModuleConstants moduleConstants) {
        absoluteEncoderOffset = moduleConstants.angleOffset;
        sparkMax = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        absoluteEncoder = sparkMax.getAbsoluteEncoder();
        absoluteEncoder.setPositionConversionFactor(1); // Ensure the output is in rotations
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.rawAbsolutePosition = Rotation2d.fromRotations(absoluteEncoder.getPosition());
        inputs.absoluteTurnPosition = inputs.rawAbsolutePosition.minus(absoluteEncoderOffset);
    }
}
