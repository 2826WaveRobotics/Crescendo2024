package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

// List class features here, including any motors, sensors, and functionality:
public class Climber extends SubsystemBase {
  private static Climber instance = null;
  public static Climber getInstance() {
      if (instance == null) {
          instance = new Climber();
      }
      return instance;
  }

  // Declare member variables here
  private CANSparkMax leftClimberMotor;
  private CANSparkMax rightClimberMotor;

  private RelativeEncoder leftClimberEncoder;
  private RelativeEncoder rightClimberEncoder;

  private Climber() {
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
