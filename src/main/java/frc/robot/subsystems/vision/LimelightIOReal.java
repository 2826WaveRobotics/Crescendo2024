package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.LimelightHelpers;

public class LimelightIOReal implements LimelightIO {
    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        inputs.pose = limelightMeasurement.pose;
        inputs.avgTagDist = limelightMeasurement.avgTagDist;
        inputs.tagCount = limelightMeasurement.tagCount;
        inputs.timestampSeconds = limelightMeasurement.timestampSeconds;

        inputs.intakeNotePresent = LimelightHelpers.getTV("limelight-intake");
        inputs.intakeNoteX = Rotation2d.fromDegrees(LimelightHelpers.getTX("limelight-intake"));
    }

    private Command flashIntakeLimelightCommand = new SequentialCommandGroup(
        new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceBlink("limelight-intake")),
        new WaitCommand(0.5),
        new InstantCommand(() -> LimelightHelpers.setLEDMode_PipelineControl("limelight-intake"))
    );

    @Override
    public void flashIntakeLimelight() {
        flashIntakeLimelightCommand.schedule();
    }
}
