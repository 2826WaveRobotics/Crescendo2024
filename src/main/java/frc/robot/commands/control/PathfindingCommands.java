package frc.robot.commands.control;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.transport.LaunchNote;
import frc.robot.commands.transport.SetLauncherAngle;
import frc.robot.commands.transport.SetLauncherSpeed;
import frc.robot.controls.SwerveAlignmentController;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.launcher.Launcher;

public class PathfindingCommands {
    private static Command pathfind(String pathName, Command... afterPathfindCommands) {
        var commandGroup = new SequentialCommandGroup();

        commandGroup.addRequirements(Swerve.getInstance(), Launcher.getInstance());

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile(pathName),
            Constants.Swerve.pathfindingConstraints,
            0.1 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        
        commandGroup.addCommands(
            new InstantCommand(() -> SwerveAlignmentController.getInstance().reset()),
            new InstantCommand(() -> DriveCommands.setPathfinding(true)),
            new ParallelCommandGroup(
                new SetLauncherAngle(65),
                new SetLauncherSpeed(1500, false),
                pathfindingCommand
            ),
            new InstantCommand(() -> DriveCommands.setPathfinding(false))
        );
        commandGroup.addCommands(afterPathfindCommands);

        return commandGroup.finallyDo(() -> DriveCommands.setPathfinding(false));
    }

    public static Command pathfindToAmpAndLaunch() {
        return pathfind("Internal_AmpLineup", new LaunchNote());
    }
    
    public static Command pathfindToSpeakerAndLaunch() {
        return pathfind("Internal_SpeakerLineup", new LaunchNote());
    }
}
