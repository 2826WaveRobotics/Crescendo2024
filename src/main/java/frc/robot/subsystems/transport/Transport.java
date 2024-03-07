package frc.robot.subsystems.transport;

import java.util.HashMap;
import java.util.HashSet;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The transport subsystem.  This subsystem controls the transport of notes from the intake to the shooter.
 * The transport is a state machine with predefined transitions.
 * Higher priority states can't transition to lower priority states;
 * for example, you can't transition from MovingNote to IntakingNote or EjectingNote to MovingNote.
 */
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
   * A state the transport can be in.  
   * The transport is a state machine with predefined transitions.
   * Higher priority states can't transition to lower priority states;
   * for example, you can't transition from MovingNote to IntakingNote or EjectingNote to MovingNote.
   */
  public enum TransportState {
    IntakingNote,
    MovingNote,
    EjectingNote,
    OperatorOverride,
    LaunchingNote,
    TrapEjectNote,
    /**
     * Used when we do a "center sweep" to put notes on our side in auto.
     * Different from IntakingNote because there are no normally-run transitions from this state,
     * so it stays on until the end of auto or we request a transition to "stopped".
     */
    SweepTransport,
    Stopped
  }

  /**
   * The current state of the transport.
   * The transport is a state machine with predefined transitions.
   * Higher priority states can't transition to lower priority states;
   * for example, you can't transition from MovingNote to IntakingNote or EjectingNote to MovingNote.
   */
  private TransportState transportState = TransportState.Stopped;

  /**
   * The operator can override the transport to run at a specific speed.  
   * This is the speed that the transport will run at when being overriden by an operator, in meters per second.
   */
  private double operatorOverrideSpeedMetersPerSecond = 0;
  /**
   * Sets the speed that the transport will run at when being overriden by an operator, in meters per second.
   * @param speedMetersPerSecond
   */
  public void setOperatorOverrideSpeedMetersPerSecond(double speedMetersPerSecond) {
    operatorOverrideSpeedMetersPerSecond = speedMetersPerSecond;
  }

  /** Valid state transitions, in the order (from, to) */
  private HashSet<Pair<TransportState, TransportState>> validStateTransitions = new HashSet<Pair<TransportState, TransportState>>();
  {
    // Standard note path transitions
    validStateTransitions.add(new Pair<>(TransportState.Stopped, TransportState.IntakingNote));
    validStateTransitions.add(new Pair<>(TransportState.IntakingNote, TransportState.MovingNote));
    validStateTransitions.add(new Pair<>(TransportState.MovingNote, TransportState.Stopped));
    validStateTransitions.add(new Pair<>(TransportState.Stopped, TransportState.LaunchingNote));
    validStateTransitions.add(new Pair<>(TransportState.LaunchingNote, TransportState.Stopped));
    
    validStateTransitions.add(new Pair<>(TransportState.MovingNote, TransportState.LaunchingNote));

    validStateTransitions.add(new Pair<>(TransportState.Stopped, TransportState.TrapEjectNote));
    validStateTransitions.add(new Pair<>(TransportState.TrapEjectNote, TransportState.Stopped));

    // Operator override transitions
    validStateTransitions.add(new Pair<>(TransportState.Stopped, TransportState.OperatorOverride));
    validStateTransitions.add(new Pair<>(TransportState.EjectingNote, TransportState.OperatorOverride));
    validStateTransitions.add(new Pair<>(TransportState.IntakingNote, TransportState.OperatorOverride));
    validStateTransitions.add(new Pair<>(TransportState.LaunchingNote, TransportState.OperatorOverride));
    validStateTransitions.add(new Pair<>(TransportState.MovingNote, TransportState.OperatorOverride));
    validStateTransitions.add(new Pair<>(TransportState.OperatorOverride, TransportState.Stopped));
    validStateTransitions.add(new Pair<>(TransportState.SweepTransport, TransportState.OperatorOverride));

    // Ejecting note transitions
    validStateTransitions.add(new Pair<>(TransportState.IntakingNote, TransportState.EjectingNote));
    validStateTransitions.add(new Pair<>(TransportState.MovingNote, TransportState.EjectingNote));
    validStateTransitions.add(new Pair<>(TransportState.Stopped, TransportState.EjectingNote));
    validStateTransitions.add(new Pair<>(TransportState.LaunchingNote, TransportState.EjectingNote));
    validStateTransitions.add(new Pair<>(TransportState.EjectingNote, TransportState.Stopped));

    // Sweep transport transitions
    validStateTransitions.add(new Pair<>(TransportState.Stopped, TransportState.SweepTransport));
    validStateTransitions.add(new Pair<>(TransportState.SweepTransport, TransportState.Stopped));
    validStateTransitions.add(new Pair<>(TransportState.SweepTransport, TransportState.EjectingNote));
  }

  /**
   * Attempts to transition to a new state.  
   * Some state transitions don't work. For example, you can't transition from MovingNote to IntakingNote or EjectingNote to MovingNote.
   * @param newState
   */
  public void attemptTransitionToState(TransportState newState) {
    if (validStateTransitions.contains(new Pair<>(transportState, newState))) {
      transportState = newState;
    }
  }

  /**
   * Gets the current state of the transport.
   * @return
   */
  public TransportState getCurrentState() {
    return transportState;
  }

  /**
   * The speed for each state, in meters per second.  
   * Speeds are how fast the belt/edge of intake wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.
   * The order of the pair is (upper transport speed, intake speed).
   */
  private HashMap<TransportState, Pair<Double, Double>> stateSpeeds = new HashMap<>();
  {
    stateSpeeds.put(TransportState.Stopped, new Pair<>(0.0, 0.0));
    stateSpeeds.put(TransportState.IntakingNote, new Pair<>(Constants.Intake.intakeSpeed, Constants.Intake.intakeSpeed));
    stateSpeeds.put(TransportState.MovingNote, new Pair<>(Constants.Intake.intakeSpeed, 0.));
    stateSpeeds.put(TransportState.EjectingNote, new Pair<>(-Constants.Transport.ejectNoteSpeed, -Constants.Transport.ejectNoteSpeed));
    stateSpeeds.put(TransportState.OperatorOverride, new Pair<>(0.0, 0.0)); // Manually handled
    stateSpeeds.put(TransportState.SweepTransport, new Pair<>(Constants.Intake.intakeSpeed, Constants.Intake.intakeSpeed));
    stateSpeeds.put(TransportState.LaunchingNote, new Pair<>(Constants.Transport.launchNoteTransportSpeed, 0.0));
  }

  @Override
  public void periodic() {
    // Speeds are how fast the belt/edge of intake wheels will move at, in meters per second.
    // This is effectively the speed that the note moves.  

    immediatelyUppdateSpeeds();
  }

  /**
   * Resets all transport state; called at the start of auto and teleop.
   */
  public void resetState() {
    transportState = TransportState.Stopped;
  }

  /** Immediately updates the transport motors with the new speeds. Used to reduce latency. */
  public void immediatelyUppdateSpeeds() {
    if (transportState == TransportState.OperatorOverride) {
      transportIO.setTransportSpeed(operatorOverrideSpeedMetersPerSecond, operatorOverrideSpeedMetersPerSecond);
    } else {
      var speed = stateSpeeds.get(transportState);
      transportIO.setTransportSpeed(speed.getFirst(), speed.getSecond());
    }
  }
}
