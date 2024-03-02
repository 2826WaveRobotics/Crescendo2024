package frc.lib.util;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.subsystems.drive.SwerveModule;

public class ShuffleboardContent {
    public static void initSwerveModuleShuffleboard(SwerveModule module) {
        int moduleNumber = module.moduleNumber;
        String[] modulePosition = { "FL", "FR", "BL", "BR" };
        String modulePositionAbbreviation = modulePosition[moduleNumber];
        String swerveModuleHeader = module.getModuleName(moduleNumber) + " Module";

        ShuffleboardLayout swerveModuleLayout = Shuffleboard.getTab("Swerve Module Data")
                .getLayout(swerveModuleHeader, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 0)
                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

        swerveModuleLayout.addNumber("ABS Angular Position (" + modulePositionAbbreviation + ")", module::getAbsoluteModuleAngleDegrees);
        swerveModuleLayout.addNumber("Angular Position (" + modulePositionAbbreviation + ")", module::getRelativeAngle);
        swerveModuleLayout.addNumber("Velocity (" + modulePositionAbbreviation + ")", module::getVelocity);
        swerveModuleLayout.addNumber("Drive Current (" + modulePositionAbbreviation + ")", module::getDriveMotorCurrent);
        swerveModuleLayout.addNumber("Turn Current (" + modulePositionAbbreviation + ")", module::getTurnMotorCurrent);
        swerveModuleLayout.addNumber("Offset (" + modulePositionAbbreviation + ")", module::getOffset);
    }
}
