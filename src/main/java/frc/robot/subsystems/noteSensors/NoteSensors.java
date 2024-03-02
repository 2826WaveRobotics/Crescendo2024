package frc.robot.subsystems.noteSensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteSensors extends SubsystemBase {
    private static NoteSensors instance = null;
    public static NoteSensors getInstance() {
        if (instance == null) {
            instance = new NoteSensors();
        }
        return instance;
    }

    // TODO: Change to IO class for AdvantageKit

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

    private NoteSensors() {
        // This is a singleton class.
    }

    /**
     * Gets if the through beam sensor for detecting if notes are in the intake is activated (meaning there's a note there).
     * @return
     */
    public boolean getIntakeSensorActivated() {
        return intakeSensor.get();
    }

    /**
     * Gets if the through beam sensor for detecting if the note is in position is activated (meaning there's a note there).
     * @return
     */
    public boolean getNoteInPositionSensorActivated() {
        return noteInPositionSensor.get();
    }

    /**
     * Gets if the through beam sensor for detecting if the note is transitioning to the resting position is activated
     * (meaning there's a note there).
     * @return
     */
    public boolean getNoteInTransitionSensorActivated() {
        return noteInTransitionSensor.get();
    }
}
