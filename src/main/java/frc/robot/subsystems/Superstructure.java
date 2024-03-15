package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.Consumer;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.climber.ClimberFullyDown;
import frc.robot.commands.climber.ClimberFullyUp;
import frc.robot.commands.climber.RunClimberSideDistance;
import frc.robot.commands.climber.RunClimberSideUntilStall;
import frc.robot.commands.elevator.ElevatorCommands;
import frc.robot.commands.transport.EjectNoteForTrap;
import frc.robot.controls.SwerveAlignmentController;
import frc.robot.controls.VibrationFeedback;
import frc.robot.controls.VibrationFeedback.VibrationPatternType;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.noteSensors.NoteSensors;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.Transport.TransportState;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;
    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    /**
     * The current state of the note in the robot.
     */
    public enum NoteState {
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
     * Gets the current state of the note in the robot.
     * @return
     */
    public NoteState getNoteState() {
        return currentState;
    }

    /**
     * A map from the sensor states combined as a binary value to the note state.
     */
    private HashMap<Integer, NoteState> sensorStateMap = createSensorStateHashmap();
    private HashMap<Integer, NoteState> createSensorStateHashmap() {
        HashMap<Integer, NoteState> hashmap = new HashMap<>(8);
        // [intake, transition, position] sensor order
        // Using bottom two
        hashmap.put(0b000, NoteState.NoNote);
        hashmap.put(0b001, NoteState.ReadyToLaunch);
        hashmap.put(0b010, NoteState.MovingNote); // Physically impossible in theory
        hashmap.put(0b011, NoteState.MovingNote);
        hashmap.put(0b100, NoteState.IntakingNote);
        hashmap.put(0b101, NoteState.EjectingNote);
        hashmap.put(0b110, NoteState.MovingNote);
        hashmap.put(0b111, NoteState.EjectingNote);

        // Using top two
        // hashmap.put(0b000, NoteState.NoNote);
        // hashmap.put(0b100, NoteState.IntakingNote);
        // hashmap.put(0b010, NoteState.MovingNote);
        // hashmap.put(0b110, NoteState.EjectingNote);
        // hashmap.put(0b001, NoteState.ReadyToLaunch); // Technically, launching
        // hashmap.put(0b101, NoteState.EjectingNote);
        // hashmap.put(0b011, NoteState.ReadyToLaunch);
        // hashmap.put(0b111, NoteState.EjectingNote);
        return hashmap;
    }

    /**
     * A notifier to update the note state.  
     * We use a notifier instead of updating in periodic because we want faster updates than the periodic loop.
     */
    private Notifier updateNoteStateNotifier = new Notifier(this::updateNoteState);

    /**
     * The number of note state updates we run per second.
     */
    private static final double NOTE_STATE_UPDATE_RATE = 100;

    private EventLoop noteStateEventLoop = new EventLoop();

    // I would use triggers here, but for some reason, they lead to internal scheduler ConcurrentModificationException errors that I don't want to deal with.

    /** Used for controller vibration feedback. */
    private BooleanEvent intakingNoteEvent = new BooleanEvent(noteStateEventLoop, () -> currentState == NoteState.IntakingNote)
        // .debounce(0.2, DebounceType.kFalling)
        .rising();

    private BooleanEvent movingNoteEvent = new BooleanEvent(noteStateEventLoop, () -> currentState == NoteState.MovingNote)
        // .debounce(0.2, DebounceType.kRising)
        .rising();
    
    private BooleanEvent readyToLaunchEvent = new BooleanEvent(noteStateEventLoop, () -> currentState == NoteState.ReadyToLaunch);
    
    private BooleanEvent ejectingNoteEvent = new BooleanEvent(noteStateEventLoop, () -> currentState == NoteState.EjectingNote).debounce(0.5, DebounceType.kFalling);
    private BooleanEvent startEjectNoteEvent = ejectingNoteEvent.rising();
    private BooleanEvent stopEjectNoteEvent = ejectingNoteEvent.falling();
    
    private BooleanEvent stopNoteAtLauncherEvent = readyToLaunchEvent.rising().and(ejectingNoteEvent.negate());
 
    /**
     * Updates the current state based on the sensor values.
     */
    private void updateNoteState() {
        NoteSensors noteSensorsSubsystem = NoteSensors.getInstance();

        // We don't do this in periodic because we want to synchronize the sensor reads with the superstructure state updates to avoid extra latency.
        noteSensorsSubsystem.updateSensorValues();
        
        currentState = sensorStateMap.get(
            (noteSensorsSubsystem.getNoteInPositionSensorActivated() ? 1 : 0) +
            ((noteSensorsSubsystem.getNoteInTransitionSensorActivated() ? 1 : 0) << 1) +
            ((noteSensorsSubsystem.getIntakeSensorActivated() ? 1 : 0) << 2)
        );

        noteStateEventLoop.poll();
    }

    /** Attempts to transition the transport to the given state, only if we are in a state it makes sense to in. */
    private void attemptTransitionToState(TransportState state) {
        Transport transportSubsystem = Transport.getInstance();
        
        // Do not transition if doing sweep transport or operator override
        TransportState transportState = transportSubsystem.getCurrentState();
        if (
            transportState == TransportState.SweepTransport ||
            transportState == TransportState.OperatorOverride
        ) return;

        transportSubsystem.attemptTransitionToState(state);
    }

    private Superstructure() {
        Transport transportSubsystem = Transport.getInstance();

        intakingNoteEvent.ifHigh(() -> VibrationFeedback.getInstance().runPattern(VibrationPatternType.IntakingNote));
        movingNoteEvent.ifHigh(() -> {
            attemptTransitionToState(TransportState.MovingNote);
            transportSubsystem.immediatelyUpdateSpeeds();
        });

        startEjectNoteEvent.ifHigh(() -> attemptTransitionToState(TransportState.EjectingNote));
        stopEjectNoteEvent.ifHigh(() -> attemptTransitionToState(TransportState.Stopped));
        
        stopNoteAtLauncherEvent.ifHigh(() -> {
            attemptTransitionToState(TransportState.Stopped);
            transportSubsystem.immediatelyUpdateSpeeds();
        });

        updateNoteStateNotifier.setName("SuperstructureNoteState");
        updateNoteStateNotifier.startPeriodic(1 / NOTE_STATE_UPDATE_RATE);

        if(!Constants.enableNonEssentialShuffleboard) return;
        Shuffleboard.getTab("Notes").addString("Superstructure note state", () -> currentState.toString());
    }

    Command scheduledClimbCommand = null;

    public void resetSubsystemsForAuto() {
        // Reset climber
        if(scheduledClimbCommand != null && scheduledClimbCommand.isScheduled()) scheduledClimbCommand.cancel();

        scheduledClimbCommand = new ParallelCommandGroup(
            // new SequentialCommandGroup(
                // new RetractElevator(),
                // new AngleElevatorDown()
            // ),
            new ClimberFullyDown()
        );
        scheduledClimbCommand.schedule();

        // Reset the odometry to face the current gyro angle
        Swerve.getInstance().resetRotation();
        Swerve.getInstance().autoStart();

        // Reset the transport state
        Transport.getInstance().resetState();

        SwerveAlignmentController.getInstance().reset();
    }
    
    public void resetSubsystemsForTeleop() {
        // Reset the transport state
        Transport.getInstance().resetState();
        
        SwerveAlignmentController.getInstance().reset();
    }

    public void setupClimb() {
        if(scheduledClimbCommand != null && scheduledClimbCommand.isScheduled()) scheduledClimbCommand.cancel();

        scheduledClimbCommand = new SequentialCommandGroup(
            new WaitUntilCommand(() -> getNoteState() != NoteState.IntakingNote),
            new ParallelCommandGroup(
                // ElevatorCommands.angleElevatorUp(),
                new ClimberFullyUp()
            )
        );
        scheduledClimbCommand.schedule();
    }
    public void climb() {
        if(scheduledClimbCommand != null && scheduledClimbCommand.isScheduled()) scheduledClimbCommand.cancel();

        if(getNoteState() == NoteState.MovingNote || getNoteState() == NoteState.ReadyToLaunch) {
            // Trap sequence
            scheduledClimbCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                    ElevatorCommands.extendElevator(),
                    new ClimberFullyDown()
                ),
                new EjectNoteForTrap(),
                new InstantCommand(() -> {
                    Climber.getInstance().useStallCurrentLimit();
                })
            );
        } else {
            // Non-trap sequence
            Climber climber = Climber.getInstance();
            scheduledClimbCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new RunClimberSideUntilStall(climber::setLeftSpeed, climber::getLeftMotorStallingClimb),
                    new RunClimberSideUntilStall(climber::setRightSpeed, climber::getRightMotorStallingClimb)
                ),
                new ParallelCommandGroup(
                    new RunClimberSideDistance(climber::setLeftPosition, climber::getLeftPosition, -15),
                    new RunClimberSideDistance(climber::setRightPosition, climber::getRightPosition, -15)
                ),
                new InstantCommand(() -> {
                    Climber.getInstance().useStallCurrentLimit();
                })
            );
        }
        scheduledClimbCommand.schedule();
    }

    public void unclimbStart() {
        if(scheduledClimbCommand != null && scheduledClimbCommand.isScheduled()) scheduledClimbCommand.cancel();
        
        Climber.getInstance().useClimbingCurrentLimit();

        scheduledClimbCommand = new SequentialCommandGroup(
            new ParallelCommandGroup(
                ElevatorCommands.retractElevator(),
                new ClimberFullyUp()
            ),
            ElevatorCommands.angleElevatorDown()
        );
        scheduledClimbCommand.schedule();
    }
    public void unclimbEnd() {
        if(scheduledClimbCommand != null && scheduledClimbCommand.isScheduled()) scheduledClimbCommand.cancel();
        
        scheduledClimbCommand = new ClimberFullyDown();
        scheduledClimbCommand.schedule();
    }
}
