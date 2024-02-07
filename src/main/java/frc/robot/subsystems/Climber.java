package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;

// List class features here, including any motors, sensors, and functionality:
public class Climber extends SubsystemBase {
  // Declare member variables here
  private CANSparkMax leftClimberMotor;
  private CANSparkMax rightClimberMotor;

  private RelativeEncoder leftClimberEncoder;
  private RelativeEncoder rightClimberEncoder;

  public Climber() {
    // Instantiate member variables and necessary code
    leftClimberMotor = new CANSparkMax(81, CANSparkMax.MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(82, CANSparkMax.MotorType.kBrushless);
    
    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

  }  

  @Override
  public void periodic() {
    // Insert periodic code here
    
  }
}
