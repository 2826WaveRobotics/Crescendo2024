package frc.robot.visualization;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;
import frc.robot.subsystems.launcher.Launcher;

public class LauncherVisualizer {
    private static LauncherVisualizer instance;
    public static LauncherVisualizer getInstance() {
        if (instance == null) {
            instance = new LauncherVisualizer();
        }
        return instance;
    }

    private Mechanism2d mechanism = new Mechanism2d(Constants.Swerve.wheelBase, 3);
    private MechanismRoot2d root = mechanism.getRoot("Launcher pivot", Constants.Swerve.wheelBase / 2 + 0.1, 0.35);
    private MechanismLigament2d arm = new MechanismLigament2d("Launcher arm", 0.4, -Launcher.getInstance().launcherAngle.getDegrees());

    private LauncherVisualizer() {
        root.append(arm);
    }

    public void update() {
        arm.setAngle(Launcher.getInstance().launcherAngle.times(-1));
        Logger.recordOutput("Launcher/Mechanism", mechanism);
    }
}