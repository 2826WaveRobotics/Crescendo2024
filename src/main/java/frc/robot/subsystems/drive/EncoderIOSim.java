package frc.robot.subsystems.drive;

public class EncoderIOSim implements EncoderIO {
    private final SwerveModuleIOSim simModuleIO;
    public EncoderIOSim(SwerveModuleIOSim simModuleIO) {
        this.simModuleIO = simModuleIO;
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.rawAbsolutePosition = simModuleIO.getAbsoluteTurnRotation();
        inputs.absoluteTurnPosition = simModuleIO.getAbsoluteTurnRotation();
    }
}
