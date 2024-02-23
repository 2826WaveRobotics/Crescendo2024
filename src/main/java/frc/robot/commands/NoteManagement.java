package frc.robot.commands;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Transport;

public class NoteManagement extends Command {
    /**
     * The current state of the note in the robot.
     */
    private enum NoteState {
        /**
         * Active when there's no note in the robot.
         */
        NoNote,
        /**
         * Active when we're currently intaking a note, which means there's a note in the elevator-intake interface.
         */
        IntakingNote,
        /**
         * Active when we're currently moving a note toward the resting position (right before the launcher).
         */
        MovingNote,
        /**
         * Active when we're currently ejecting a note.  
         * This happens when we accidentally intake two notes at the same time.
         */
        EjectingNote,
        /**
         * Active when we're ready to launch, meaning the note is in its resting state (right before the launcher).
         */
        ReadyToLaunch
    }

    /**
     * The current state of the note in the robot.
     */
    private NoteState currentState = NoteState.NoNote;

    /**
     * A map from the sensor states combined as a binary value to the note state.
     */
    private HashMap<Integer, NoteState> sensorStateMap = createSensorStateHashmap();
    private HashMap<Integer, NoteState> createSensorStateHashmap() {
        HashMap<Integer, NoteState> hashmap = new HashMap<>(8);
        hashmap.put(0b000, NoteState.NoNote);
        hashmap.put(0b001, NoteState.IntakingNote);
        hashmap.put(0b010, NoteState.MovingNote);
        hashmap.put(0b011, NoteState.EjectingNote);
        hashmap.put(0b100, NoteState.ReadyToLaunch);
        hashmap.put(0b101, NoteState.EjectingNote);
        hashmap.put(0b110, NoteState.MovingNote);
        hashmap.put(0b111, NoteState.EjectingNote);
        return hashmap;
    }

    /**
     * Updates the current state based on the sensor values.
     */
    private void updateNoteState() {
        currentState = sensorStateMap.get(
            (elevatorSubsystem.getIntakeSensorActivated() ? 1 : 0) +
            ((elevatorSubsystem.getNoteInTransitionSensorActivated() ? 1 : 0) << 1) +
            ((elevatorSubsystem.getNoteInPositionSensorActivated() ? 1 : 0) << 2)
        );
    }

    private Elevator elevatorSubsystem;
    private Transport transportSubsystem;
    private Trigger launchNoteTrigger;

    public NoteManagement(
            Elevator elevatorSubsystem,
            Transport transportSubsystem,
            Trigger launchNoteTrigger) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.transportSubsystem = transportSubsystem;
        this.launchNoteTrigger = launchNoteTrigger;

        addRequirements(elevatorSubsystem);
    }

    /**
     * If we're currently launching a note.
     */
    private boolean launchingNote = false;

    /**
     * If we're currently ejecting the note for trapping.
     */
    private boolean ejectingNoteForTrap = false;

    /**
     * Launches a note. This is mostly temporary logic.
     */
    public void launchNote() {
        if(currentState != NoteState.ReadyToLaunch) return;
        
        launchingNote = true;
        new WaitCommand(0.5).andThen(new InstantCommand(() -> {
            launchingNote = false;
        })).schedule();
    }

    /**
     * Ejects the current note for the trap.
     */
    public void ejectNoteForTrap() {
        // TODO: This should only work when the elevator is tilted up. We don't store that state yet, though.

        ejectingNoteForTrap = true;
        new WaitCommand(0.5).andThen(new InstantCommand(() -> {
            ejectingNoteForTrap = false;
        })).schedule();
    }

    @Override
    public void initialize() {
        Trigger ejecting = new Trigger(() -> {
            return currentState == NoteState.EjectingNote;
        });
        ejecting.onTrue(new InstantCommand(() -> transportSubsystem.ejectNote()));
        ejecting.onFalse(new WaitCommand(0.5).andThen(new InstantCommand(() -> transportSubsystem.stopEjectingNote())));

        Trigger readyToLaunch = new Trigger(() -> currentState == NoteState.ReadyToLaunch);
        readyToLaunch.onTrue(new InstantCommand(() -> transportSubsystem.setActive(false)));

        launchNoteTrigger.onTrue(new InstantCommand(this::launchNote));
    }

    @Override
    public void execute() {
        updateNoteState();
        
        double upperTransportSpeed = 0;
        if(launchingNote) {
            upperTransportSpeed = Constants.Transport.launchNoteTransportSpeed;
        } else if(ejectingNoteForTrap) {
            upperTransportSpeed = Constants.Transport.trapEjectSpeed;
        }
        // transportSubsystem.setUpperTransportSpeed(upperTransportSpeed);
    }
}
