package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

// List class features here, including any motors, sensors, and functionality:
// 2 Launcher motors to score notes
// 1 motor to aim
// 1 encoder for aiming (~30-60 degree window)
public class Launcher extends SubsystemBase {
  // Declare member variables here
  private CANSparkMax topLauncherMotor;
  private CANSparkMax bottomLauncherMotor;
  private CANSparkMax angleLauncherMotor;

  private RelativeEncoder topLauncherEncoder;
  private RelativeEncoder bottomLauncherEncoder;
  private RelativeEncoder angleLauncherEncoder;
  private CANcoder absoluteAngleLauncherEncoder;
  
  public Launcher() {
    // Instantiate member variables and necessary code
    topLauncherMotor = new CANSparkMax(71, CANSparkMax.MotorType.kBrushless);
    bottomLauncherMotor = new CANSparkMax(72, CANSparkMax.MotorType.kBrushless);
    angleLauncherMotor = new CANSparkMax(73, CANSparkMax.MotorType.kBrushless);

    topLauncherEncoder = topLauncherMotor.getEncoder();
    bottomLauncherEncoder = bottomLauncherMotor.getEncoder();
    angleLauncherEncoder = angleLauncherMotor.getEncoder();
    absoluteAngleLauncherEncoder = new CANcoder(74);

  }  


  @Override
  public void periodic() {
    // Insert periodic code here
    
  }
}
