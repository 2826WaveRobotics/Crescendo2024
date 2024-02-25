package frc.robot;

import java.sql.Driver;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.commands.NoteManagement.NoteState;

public class Lighting {
    /**
     * The possible lighting states.  
     * This enum MUST match the Arduino's enum definitions for the light states.
     */
    private static enum LightState {
        preStartState,
        autoState,
        teleopNoteIntookState,
        teleopTransportState,
        teleopNoteReadyState,
        teleopEjectingNoteState,
        teleopStaticState
    };

    /**
     * The I2C device for the lighting arduino.
     */
    private static I2C i2cLightingArduino = new I2C(Port.kMXP, 5);

    /**
     * Gets the current lighting state.
     * @return
     */
    LightState getLightingState(NoteState noteManagementState) {
        if(DriverStation.isDisabled()) return LightState.preStartState;
        if(DriverStation.isAutonomous()) return LightState.autoState;

        // Teleop states
        if(noteManagementState == NoteState.IntakingNote)  return LightState.teleopNoteIntookState;
        if(noteManagementState == NoteState.MovingNote)    return LightState.teleopTransportState;
        if(noteManagementState == NoteState.ReadyToLaunch) return LightState.teleopNoteReadyState;
        if(noteManagementState == NoteState.EjectingNote)  return LightState.teleopEjectingNoteState;

        return LightState.teleopStaticState;
    }

    /**
     * Gets a lighting update with the current lighting state.  
     * Message format is 1 byte for the light state, 1 byte for the alliance, and 1 byte for the robot speed
     * (as a ratio of the full speed).
     * Checks and casts are done on both sides.
     * 
     * @param robotSpeed The robot speed in meters per second.
     */
    byte[] getUpdate(double robotSpeed, NoteState noteManagementState) {
        byte[] data = new byte[3];
        data[0] = (byte)getLightingState(noteManagementState).ordinal();

        // This value matches the Arduino's enum definitions for the alliance.
        data[1] = (byte)(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 1);
        // 5.4 meters per second is our maximum speed.
        // This is just a rough conversion to 0-255, which is what we send to the Arduino.
        data[2] = (byte)(robotSpeed / 5.4 * 255);
        return data;
    }

    /**
     * Updates the arduino with the state it requires.
     * This is run periodically, no matter if the robot is enabled, by the scheduler.
     * 
     * @param robotSpeed The robot speed in meters per second.
     */
    void periodic(double robotSpeed, NoteState noteManagementState) {
        i2cLightingArduino.writeBulk(getUpdate(robotSpeed, noteManagementState));
    }
}
