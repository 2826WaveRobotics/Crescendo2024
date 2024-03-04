package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.lib.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private static Limelight instance = null;
  public static Limelight getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance = new Limelight(new LimelightIOReal());
          return instance;
        default:
          // Replayed or sim robot, disable IO implementations
          instance = new Limelight(new LimelightIO() { });
          return instance;
      }
    }
    return instance;
  }

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  
  private LimelightIO limelightIO;
  private LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

  private Limelight(LimelightIO limelightIO) {
    this.limelightIO = limelightIO;
  }

  private void updateOdometryPoseFromVisionMeasurements() {
    LimelightHelpers.PoseEstimate poseEstimateData = inputs.poseEstimateData;
    if(poseEstimateData == null) return;
    
    if(poseEstimateData.tagCount <= 2) return;
    if(poseEstimateData.avgTagDist > 5) return;

    // Scale the vision measurement expected standard deviation exponentially by the distance 
    double standardDeviationScalar = Math.max(0.01, 0.2 * Math.pow(1.5, poseEstimateData.avgTagDist) - 0.03);

    Pose2d pose = poseEstimateData.pose;

    Swerve swerve = Swerve.getInstance();
    // If the vision estimate is more than 1.5 meters away from the odometry estimate, discard the vision estimate
    if (swerve.getPose().getTranslation().getDistance(pose.getTranslation()) > 1.5) {
      return;
    }

    swerve.addVisionMeasurement(pose, poseEstimateData.latency, standardDeviationScalar);
  }

  @Override
  public void periodic() {
    limelightIO.updateInputs(inputs);

    updateOdometryPoseFromVisionMeasurements();
  }
}
