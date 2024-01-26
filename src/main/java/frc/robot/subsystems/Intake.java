package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// List class features here, including any motors, sensors, and functionality:
// Two Motors for the intake
// Primary motor has first contact with the Note.  Secondary controls the rollers behind the primary motor.
// This is the intake subsystem where the intake is fully internal in the robot
public class Intake extends SubsystemBase {
  // Declare member variables here
  private CANSparkMax primaryIntakeMotor;
  private CANSparkMax secondaryIntakeMotor;

  private RelativeEncoder primaryIntakeEncoder;
  private RelativeEncoder secondaryIntakeEncoder;

  public Intake() {
    // Instantiate member variables and necessary code
    primaryIntakeMotor = new CANSparkMax(51, CANSparkMax.MotorType.kBrushless);
    secondaryIntakeMotor = new CANSparkMax(52, CANSparkMax.MotorType.kBrushless);
    
    primaryIntakeEncoder = primaryIntakeMotor.getEncoder();
    secondaryIntakeEncoder = secondaryIntakeMotor.getEncoder();

  }  

  @Override
  public void periodic() {
    // Insert periodic code here
    
  }
}
