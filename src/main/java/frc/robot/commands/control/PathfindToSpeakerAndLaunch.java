package frc.robot.commands.control;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.transport.LaunchNote;
import frc.robot.commands.transport.SetLauncherAngle;
import frc.robot.controls.SwerveAlignmentController;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.launcher.Launcher;

public class PathfindToSpeakerAndLaunch extends SequentialCommandGroup {
    public PathfindToSpeakerAndLaunch() {
        addRequirements(Swerve.getInstance(), Launcher.getInstance());

        // Load the path we want to pathfind to and follow
        PathPlannerPath path = PathPlannerPath.fromPathFile("Internal_SpeakerLineup");

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                Constants.Swerve.pathfindingConstraints,
                1.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        SwerveAlignmentController.getInstance().reset();

        addCommands(
            new ParallelCommandGroup(
                new SetLauncherAngle(55),
                new InstantCommand(() -> Launcher.getInstance().setLauncherSpeed(4300, true)),
                pathfindingCommand
            ),
            new LaunchNote()
        );
    }
}
