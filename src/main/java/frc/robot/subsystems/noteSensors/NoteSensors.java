package frc.robot.subsystems.noteSensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.noteSensors.NoteSensorIO;

public class NoteSensors extends SubsystemBase {
  private static NoteSensors instance = null;
  public static NoteSensors getInstance() {
    if (instance == null) {
        switch (Constants.currentMode) {
          case REAL:
            instance = new NoteSensors(new NoteSensorIOReal());
            return instance;
          case REPLAY:
            instance = new NoteSensors(new NoteSensorIO() {});
            return instance;
          case SIM:
            instance = new NoteSensors(new NoteSensorIOSim());
            return instance;
        }
    }
    return instance;
  }

  private final NoteSensorIO noteSensorIO;
  private final NoteSensorIOInputsAutoLogged inputs = new NoteSensorIOInputsAutoLogged();

  private NoteSensors(NoteSensorIO noteSensorIO) {
    this.noteSensorIO = noteSensorIO;
  }

  @Override
  public void periodic() {
    noteSensorIO.updateInputs(inputs);
  }

  /**
   * Gets if the through beam sensor for detecting if notes are in the intake is activated (meaning there's a note there).
   * @return
   */
  public boolean getIntakeSensorActivated() {
    return inputs.intakeSensorActivated;
  }

  /**
   * Gets if the through beam sensor for detecting if the note is in position is activated (meaning there's a note there).
   * @return
   */
  public boolean getNoteInPositionSensorActivated() {
    return inputs.noteInPositionSensorActivated;
  }

  /**
   * Gets if the through beam sensor for detecting if the note is transitioning to the resting position is activated
   * (meaning there's a note there).
   * @return
   */
  public boolean getNoteInTransitionSensorActivated() {
    return inputs.noteInTransitionSensorActivated;
  }
}
