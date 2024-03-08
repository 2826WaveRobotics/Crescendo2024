package frc.robot.subsystems.transport;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

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

  // private class TransportStatePair {
  //   TransportState from;
  //   TransportState to;

  //   TransportStatePair(TransportState from, TransportState to) {
  //     this.from = from;
  //     this.to = to;
  //   }

  //   @Override
  //   public int hashCode() {
  //       return from.ordinal() + to.ordinal() * 100;
  //   }
  // }
    
  // /** Valid state transitions, in the order (from, to) */
  // private Set<TransportStatePair> validStateTransitions = Collections.synchronizedSet(new HashSet<>());
  // {
  //   // Standard note path transitions
  //   validStateTransitions.add(new TransportStatePair(TransportState.Stopped, TransportState.IntakingNote));
  //   validStateTransitions.add(new TransportStatePair(TransportState.IntakingNote, TransportState.MovingNote));
  //   validStateTransitions.add(new TransportStatePair(TransportState.MovingNote, TransportState.Stopped));
  //   validStateTransitions.add(new TransportStatePair(TransportState.Stopped, TransportState.LaunchingNote));
  //   validStateTransitions.add(new TransportStatePair(TransportState.LaunchingNote, TransportState.Stopped));
    
  //   validStateTransitions.add(new TransportStatePair(TransportState.MovingNote, TransportState.LaunchingNote));

  //   validStateTransitions.add(new TransportStatePair(TransportState.Stopped, TransportState.TrapEjectNote));
  //   validStateTransitions.add(new TransportStatePair(TransportState.TrapEjectNote, TransportState.Stopped));

  //   // Operator override transitions
  //   validStateTransitions.add(new TransportStatePair(TransportState.Stopped, TransportState.OperatorOverride));
  //   validStateTransitions.add(new TransportStatePair(TransportState.EjectingNote, TransportState.OperatorOverride));
  //   validStateTransitions.add(new TransportStatePair(TransportState.IntakingNote, TransportState.OperatorOverride));
  //   validStateTransitions.add(new TransportStatePair(TransportState.LaunchingNote, TransportState.OperatorOverride));
  //   validStateTransitions.add(new TransportStatePair(TransportState.MovingNote, TransportState.OperatorOverride));
  //   validStateTransitions.add(new TransportStatePair(TransportState.OperatorOverride, TransportState.Stopped));
  //   validStateTransitions.add(new TransportStatePair(TransportState.SweepTransport, TransportState.OperatorOverride));

  //   // Ejecting note transitions
  //   validStateTransitions.add(new TransportStatePair(TransportState.IntakingNote, TransportState.EjectingNote));
  //   validStateTransitions.add(new TransportStatePair(TransportState.MovingNote, TransportState.EjectingNote));
  //   validStateTransitions.add(new TransportStatePair(TransportState.Stopped, TransportState.EjectingNote));
  //   validStateTransitions.add(new TransportStatePair(TransportState.LaunchingNote, TransportState.EjectingNote));
  //   validStateTransitions.add(new TransportStatePair(TransportState.EjectingNote, TransportState.Stopped));

  //   // Sweep transport transitions
  //   validStateTransitions.add(new TransportStatePair(TransportState.Stopped, TransportState.SweepTransport));
  //   validStateTransitions.add(new TransportStatePair(TransportState.SweepTransport, TransportState.Stopped));
  //   validStateTransitions.add(new TransportStatePair(TransportState.SweepTransport, TransportState.EjectingNote));
  // }

  /**
   * Attempts to transition to a new state.  
   * Some state transitions don't work. For example, you can't transition from MovingNote to IntakingNote or EjectingNote to MovingNote.
   * @param newState
   */
  public void attemptTransitionToState(TransportState newState) {
    transportState = newState;
  }

  /**
   * Gets the current state of the transport.
   * @return
   */
  public TransportState getCurrentState() {
    return transportState;
  }

  @Override
  public void periodic() {
    // Speeds are how fast the belt/edge of intake wheels will move at, in meters per second.
    // This is effectively the speed that the note moves.  

    immediatelyUpdateSpeeds();
  }

  /**
   * Resets all transport state; called at the start of auto and teleop.
   */
  public void resetState() {
    transportState = TransportState.Stopped;
  }

  /** Immediately updates the transport motors with the new speeds. Used to reduce latency. */
  public void immediatelyUpdateSpeeds() {
    if (transportState == TransportState.OperatorOverride) {
      transportIO.setTransportSpeed(operatorOverrideSpeedMetersPerSecond, operatorOverrideSpeedMetersPerSecond);
    } else {
      switch(transportState) {
        case IntakingNote:
          transportIO.setTransportSpeed(Constants.Intake.intakeSpeed, Constants.Intake.intakeSpeed);
          break;
        case MovingNote:
          transportIO.setTransportSpeed(Constants.Intake.intakeSpeed, 0.);
          break;
        case EjectingNote:
          transportIO.setTransportSpeed(-Constants.Transport.ejectNoteSpeed, -Constants.Transport.ejectNoteSpeed);
          break;
        case SweepTransport:
          transportIO.setTransportSpeed(Constants.Intake.intakeSpeed, Constants.Intake.intakeSpeed);
          break;
        case LaunchingNote:
          transportIO.setTransportSpeed(Constants.Transport.launchNoteTransportSpeed, 0.0);
          break;
        default:
          transportIO.setTransportSpeed(0., 0.);
          break;
      }
    }
  }
}
