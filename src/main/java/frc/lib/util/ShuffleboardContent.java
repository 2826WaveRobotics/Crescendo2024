package frc.lib.util;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;

public class ShuffleboardContent {
    public static void initSwerveModuleShuffleboard(SwerveModule module, SwerveModuleIOSparkMax moduleIO) {
        int moduleNumber = module.moduleIndex;
        String[] modulePosition = { "FL", "FR", "BL", "BR" };
        String modulePositionAbbreviation = modulePosition[moduleNumber];
        String swerveModuleHeader = module.getModuleName() + " Module";

        ShuffleboardLayout swerveModuleLayout = Shuffleboard.getTab("Swerve Module Data")
                .getLayout(swerveModuleHeader, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 0)
                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

        swerveModuleLayout.addNumber("CANcoder Position (" + modulePositionAbbreviation + ")", moduleIO::getCANcoderPositionDegrees);
        swerveModuleLayout.addNumber("Drive Current (" + modulePositionAbbreviation + ")", moduleIO::getDriveMotorCurrent);
        swerveModuleLayout.addNumber("Turn Current (" + modulePositionAbbreviation + ")", moduleIO::getTurnMotorCurrent);
    }
}
