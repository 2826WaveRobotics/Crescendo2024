package frc.robot.subsystems.noteSensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class NoteSensorIOReal implements NoteSensorIO {
  /**
   * The through beam sensor for detecting if notes are in the intake.
   */
  private DigitalInput intakeSensor = new DigitalInput(Constants.Elevator.intakeSensorDIOPort);
  /**
   * The through beam sensor detecting if the note is in position.
   */
  private DigitalInput noteInPositionSensor = new DigitalInput(Constants.Elevator.noteInPositionSensorDIOPort);
  /**
   * The through beam sensor detecting if the note is transitioning to the resting position.
   */
  private DigitalInput noteInTransitionSensor = new DigitalInput(Constants.Elevator.noteInTransitionSensorDIOPort);

  @Override
  public void updateInputs(NoteSensorIOInputs inputs) {
    inputs.intakeSensorActivated = intakeSensor.get();
    inputs.noteInPositionSensorActivated = noteInPositionSensor.get();
    inputs.noteInTransitionSensorActivated = noteInTransitionSensor.get();
  }
}
