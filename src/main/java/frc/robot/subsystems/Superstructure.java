package frc.robot.subsystems;

import java.util.HashMap;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.climber.ClimberFullyDown;
import frc.robot.commands.climber.ClimberFullyUp;
import frc.robot.commands.climber.RunClimberSideDistance;
import frc.robot.commands.climber.RunClimberSideUntilStall;
import frc.robot.commands.elevator.AngleElevatorDown;
import frc.robot.commands.elevator.AngleElevatorUp;
import frc.robot.commands.elevator.ExtendElevator;
import frc.robot.commands.elevator.RetractElevator;
import frc.robot.commands.transport.EjectNoteForTrap;
import frc.robot.commands.transport.LaunchNote;
import frc.robot.controls.SwerveAlignmentController;
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
        // [position, transition, intake] sensor order
        hashmap.put(0b000, NoteState.NoNote);
        hashmap.put(0b001, NoteState.IntakingNote);
        hashmap.put(0b010, NoteState.MovingNote);
        hashmap.put(0b011, NoteState.EjectingNote);
        hashmap.put(0b100, NoteState.MovingNote);
        hashmap.put(0b101, NoteState.EjectingNote);
        hashmap.put(0b110, NoteState.ReadyToLaunch);
        hashmap.put(0b111, NoteState.EjectingNote);
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
    private Trigger ejectingNoteEvent = new Trigger(noteStateEventLoop, () -> currentState == NoteState.EjectingNote);
    private Trigger readyToLaunchEvent = new Trigger(noteStateEventLoop, () -> currentState == NoteState.ReadyToLaunch);

    private Lock sensorMapLock = new ReentrantLock();
    
    /**
     * Updates the current state based on the sensor values.
     */
    private void updateNoteState() {
        NoteSensors noteSensorsSubsystem = NoteSensors.getInstance();

        // We don't do this in periodic because we want to synchronize the sensor reads with the superstructure state updates to avoid extra latency.
        noteSensorsSubsystem.updateSensorValues();
        
        sensorMapLock.lock();
        currentState = sensorStateMap.get(
            (noteSensorsSubsystem.getIntakeSensorActivated() ? 1 : 0) +
            ((noteSensorsSubsystem.getNoteInTransitionSensorActivated() ? 1 : 0) << 1) +
            ((noteSensorsSubsystem.getNoteInPositionSensorActivated() ? 1 : 0) << 2)
        );
        sensorMapLock.unlock();

        noteStateEventLoop.poll();

        Transport transportSubsystem = Transport.getInstance();
        readyToLaunchEvent.onTrue(new InstantCommand(() -> {
            transportSubsystem.attemptTransitionToState(TransportState.Stopped);
            transportSubsystem.immediatelyUpdateSpeeds();
        }));
        
        ejectingNoteEvent.onTrue(new InstantCommand(() -> transportSubsystem.attemptTransitionToState(TransportState.EjectingNote)));
        ejectingNoteEvent.onFalse(new WaitCommand(0.5).andThen(() -> transportSubsystem.attemptTransitionToState(TransportState.Stopped)));
    }

    private Superstructure() {
        updateNoteStateNotifier.startPeriodic(1 / NOTE_STATE_UPDATE_RATE);

        Shuffleboard.getTab("Note sensors").addString("Superstructure note state", () -> currentState.toString());
    }

    /**
     * Launches a note.
     */
    public void launchNote() {
        new LaunchNote().schedule();
    }

    /**
     * Ejects the current note for the trap.
     */
    public void ejectNoteForTrap() {
        new EjectNoteForTrap().schedule();
    }

    Command scheduledClimbCommand = null;

    public void resetSubsystemsForAuto() {
        // Reset climber
        if(scheduledClimbCommand != null && scheduledClimbCommand.isScheduled()) scheduledClimbCommand.cancel();

        // scheduledClimbCommand = new ParallelCommandGroup(
        //     new SequentialCommandGroup(
        //         new RetractElevator(),
        //         new AngleElevatorDown()
        //     ),
        //     new ClimberFullyDown()
        // );
        // scheduledClimbCommand.schedule();

        // Reset the odometry to face the current gyro angle
        Swerve.getInstance().resetRotation();

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
                new AngleElevatorUp(),
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
                    new ExtendElevator(),
                    new ClimberFullyDown()
                ),
                new InstantCommand(this::ejectNoteForTrap)
            );
        } else {
            // Non-trap sequence
            Climber climber = Climber.getInstance();
            scheduledClimbCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new RunClimberSideUntilStall(climber::setLeftSpeed, climber::getLeftMotorStalling),
                    new RunClimberSideUntilStall(climber::setRightSpeed, climber::getRightMotorStalling)
                ),
                new ParallelCommandGroup(
                    new RunClimberSideDistance(climber::setLeftPosition, climber::getLeftPosition, -15),
                    new RunClimberSideDistance(climber::setRightPosition, climber::getRightPosition, -15)
                )
            );
        }
        scheduledClimbCommand.schedule();
    }

    public void unclimbStart() {
        if(scheduledClimbCommand != null && scheduledClimbCommand.isScheduled()) scheduledClimbCommand.cancel();
        
        scheduledClimbCommand = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RetractElevator(),
                new ClimberFullyUp()
            ),
            new AngleElevatorDown()
        );
        scheduledClimbCommand.schedule();
    }
    public void unclimbEnd() {
        if(scheduledClimbCommand != null && scheduledClimbCommand.isScheduled()) scheduledClimbCommand.cancel();
        
        scheduledClimbCommand = new ClimberFullyDown();
        scheduledClimbCommand.schedule();
    }
}
