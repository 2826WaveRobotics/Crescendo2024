package frc.robot.oi;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.subsystems.SwerveModule;

public class ShuffleboardContent {

    public static void initSwerveModuleShuffleboard(SwerveModule module) {
        int moduleNumber = module.moduleNumber;
        String[] modulePosition = { "FL", "FR", "BL", "BR" };
        String modulePositionAbbreviation = modulePosition[moduleNumber];
        String swerveModuleHeader = module.getModuleName(moduleNumber) + " Module (ID " + moduleNumber + ")";

        ShuffleboardLayout swerveModuleLayout = Shuffleboard.getTab("Swerve Module Data")
                .getLayout(swerveModuleHeader, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 0)
                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

        swerveModuleLayout.addNumber("ABS Angular Position (" + modulePositionAbbreviation + ")",
                () -> module.getAbsoluteModuleAngleDegrees());
        swerveModuleLayout.addNumber("Angular Position (" + modulePositionAbbreviation + ")",
                () -> module.getRelativeAngle());
        swerveModuleLayout.addNumber("Velocity (" + modulePositionAbbreviation + ")",
                () -> module.getVelocity());
        swerveModuleLayout.addNumber("Drive Current (" + modulePositionAbbreviation + ")", 
                () -> module.getDriveMotorCurrent());
        swerveModuleLayout.addNumber("Turn Current (" + modulePositionAbbreviation + ")", 
                () -> module.getTurnMotorCurrent());
        swerveModuleLayout.addNumber("Offset (" + modulePositionAbbreviation + ")", 
                () -> module.getOffset());
        // swerveModuleLayout.addString("Magnetic Field (" + modulePositionAbbreviation + ")", 
        //         () -> module.getMagnetFieldStrength().toString());

        // FONDY FIRE code example
        // int m_moduleNumber = m_sm.m_locationIndex;
        // String abrev = m_sm.modAbrev[m_moduleNumber];
        // String canCoderLayout = m_sm.getModuleName(m_moduleNumber) + " CanCoder";

        // ShuffleboardLayout coderLayout = Shuffleboard.getTab("CanCoders")
        // .getLayout(canCoderLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber
        // * 2, 0)
        // .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

        // coderLayout.addNumber("Position" + abrev,
        // () -> m_sm.m_turnCANcoder.getPosition());
        // coderLayout.addNumber("Abs Position" + abrev,
        // () -> m_sm.m_turnCANcoder.getAbsolutePosition());
        // coderLayout.addNumber("Velocity" + abrev,
        // () -> m_sm.m_turnCANcoder.getVelocity());
        // coderLayout.addString(" MagField " + abrev,
        // () -> m_sm.m_turnCANcoder.getMagnetFieldStrength().toString());

        // coderLayout.addNumber("Bus Volts" + abrev,
        // () -> m_sm.m_turnCANcoder.getBusVoltage());

        // coderLayout.addNumber("Abs Offset" + abrev, () -> m_sm.m_turnEncoderOffset);

        // coderLayout.addNumber("Firmware#" + abrev,
        // () -> m_sm.m_turnCANcoder.getFirmwareVersion());

    }
}
