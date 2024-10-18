package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.config.CTREConfigs;
import frc.lib.config.SwerveModuleConstants;

public class EncoderIOCANCoder implements EncoderIO {
    private static final CTREConfigs ctreConfigs = new CTREConfigs();

    private final Rotation2d absoluteEncoderOffset;
    private final CANcoder cancoder;
    private final StatusSignal<Double> turnAbsolutePosition;

    public EncoderIOCANCoder(SwerveModuleConstants moduleConstants) {
        absoluteEncoderOffset = moduleConstants.angleOffset;
        
        cancoder = new CANcoder(moduleConstants.encoderInputID);
        cancoder.getConfigurator().apply(ctreConfigs.swerveCanCoderConfig);
        cancoder.optimizeBusUtilization();
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnAbsolutePosition.setUpdateFrequency(20.0);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        turnAbsolutePosition.refresh();

        inputs.rawAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.absoluteTurnPosition = inputs.rawAbsolutePosition.minus(absoluteEncoderOffset);
    }
}
