package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// List class features here, including any motors, sensors, and functionality:
// Two Motors for the intake
// Primary motor has first contact with the Note.  Secondary controls the rollers behind the primary motor.
// This is the intake subsystem where the intake is fully internal in the robot
public class Transport extends SubsystemBase {
  private static Transport instance = null;
  public static Transport getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance = new Transport(new TransportIOReal());
          return instance;
        default:
          instance = new Transport(new TransportIO() {});
          return instance;
      }
    }
    return instance;
  }

  private TransportIO transportIO;

  private Transport(TransportIO transportIO) {
    this.transportIO = transportIO;
  }

  /**
   * The intake wheel speeds.
   * Speeds are how fast the belt/edge of intake wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.  
   * If the intake is running faster than the transport, the transport will run at the intake speed.
   */
  double intakeSpeedMetersPerSecond = 0;
  /**
   * The upper transport wheel speed.
   * Speeds are how fast the belt/edge of intake wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.  
   * If the intake is running faster than the transport, the transport will run at the intake speed.
   */
  double transportSpeedMetersPerSecond = 0; 
    
  /**
   * Returns if the intake is currently active, meaning the speed is nonzero.
   * @return
   */
  public boolean isActive() {
    return intakeSpeedMetersPerSecond != 0;
  }

  /**
   * Sets the intake active. False sets the speed to 0 and true sets the speed to `Constants.Intake.intakeSpeed`.
   * @param active
   */
  public void setActive(boolean active) {
    setIntakeSpeed(active ? Constants.Intake.intakeSpeed : 0);
  }
  
  /**
   * Sets the speed of the top transport motor. If the intake is faster, this is overriden to avoid ripping
   * notes and/or the robot apart.
   * @param speed The speed the the edge of the wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.
   */
  public void setUpperTransportSpeed(double speed) {
    transportSpeedMetersPerSecond = speed;
  }

  /**
   * Sets the intake motors to the given speed.
   * @param speed The speed the the belt/edge of intake wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.
   */
  public void setIntakeSpeed(double speedInMetersPerSecond) {
    intakeSpeedMetersPerSecond = speedInMetersPerSecond;
  }

  public boolean isEjectingNote = false;
  /**
   * Starts ejecting a note.  
   * This occurs when there are multiple notes in the intake at once.
   */
  public void ejectNote() { isEjectingNote = true; }
  /**
   * Stops ejecting a note.  
   * This occurs when there are multiple notes in the intake at once.
   */
  public void stopEjectingNote() { isEjectingNote = false; }

  @Override
  public void periodic() {
    double intakeSpeed = isEjectingNote ? -Constants.Intake.ejectSpeedMetersPerSecond : intakeSpeedMetersPerSecond;
    double transportSpeed = isEjectingNote ? -Constants.Intake.ejectSpeedMetersPerSecond : transportSpeedMetersPerSecond;
    transportIO.setTransportSpeed(transportSpeed, intakeSpeed);
  }
}
