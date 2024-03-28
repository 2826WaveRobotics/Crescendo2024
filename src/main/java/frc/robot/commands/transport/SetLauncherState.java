package frc.robot.commands.transport;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

/**
 * A command that sets the launcher state and finishes when both the angle and speed are at the target values.
 */
public class SetLauncherState extends ParallelCommandGroup {
  /**
   * Creates a new SetLauncherState.
   * @param state The launcher state
   */
  public SetLauncherState(LauncherState state) {
    addCommands(
      new SetLauncherAngle(state.angleDegrees),
      new SetLauncherSpeed(state.speed, state.adjustForSpeaker)
    );
  }
}
