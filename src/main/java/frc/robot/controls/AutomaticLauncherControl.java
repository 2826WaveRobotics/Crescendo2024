package frc.robot.controls;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.controls.SwerveAlignmentController.AlignmentMode;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class AutomaticLauncherControl {
  private static AutomaticLauncherControl instance = null;
  public static AutomaticLauncherControl getInstance() {
      if (instance == null) {
          instance = new AutomaticLauncherControl();
      }
      return instance;
  }
  
  private AutomaticLauncherControl() {
    // This is a singleton class.
  }
  
  private ArrayList<LauncherState> launcherStateGrid = new ArrayList<>();
  private int grid_x_size;
  private int grid_y_size;
  private float minx;
  private float maxx;
  private float miny;
  private float maxy;
  {
    try (BufferedReader br =
    new BufferedReader(new FileReader(new File(Filesystem.getDeployDirectory(), "launcherData.txt")))
    ) {
      // The custom file format is pretty simple; see misc/launcherAimDataProcessing.ipynb for more details
      
      String[] sizes = br.readLine().split(":");
      grid_x_size = Integer.parseInt(sizes[0]);
      grid_y_size = Integer.parseInt(sizes[1]);
      minx = Float.parseFloat(sizes[2]);
      maxx = Float.parseFloat(sizes[3]);
      miny = Float.parseFloat(sizes[4]);
      maxy = Float.parseFloat(sizes[5]);
      
      String line;
      while ((line = br.readLine()) != null) {
        if(line.length() == 0) continue; // Skip empty lines -- the last line will be empty
        String[] parts = line.split(",");
        float speed = Float.parseFloat(parts[0]);
        float angle = Float.parseFloat(parts[1]);
        launcherStateGrid.add(new LauncherState(speed, angle));
      }
    } catch(IOException ioException) {
      DriverStation.reportWarning("WARNING: IO exception while reading automatic launcher control file: " + ioException.getLocalizedMessage(), false);
    }
  }
  
  private LauncherState getGridPoint(int x, int y) {
    x = Math.min(Math.max(x, 0), grid_x_size - 1);
    y = Math.min(Math.max(y, 0), grid_y_size - 1);
    
    return launcherStateGrid.get(y * grid_x_size + x);
  }
  
  private LauncherState getLauncherStateLookupTable() {
    Swerve swerve = Swerve.getInstance();
    
    // Find the closest 4 points in the lookup table and interpolate between them
    // Adjust for the robot's speed since by the time we reach the target, the robot will have moved
    double lookaheadDistance = swerve.getRobotSpeed() * 0.2; // 0.2 is abritrary; let's say it takes 200ms to adjust the launcher
    Pose2d pose = swerve.getPose().transformBy(new Transform2d(lookaheadDistance, 0.0, new Rotation2d()));
    Translation2d currentTranslation = pose.getTranslation();
    
    // We find the 4 closest points in the grid and bilinearly interpolate between them
    int x1 = (int) Math.floor((currentTranslation.getX() - minx) / (maxx - minx) * (grid_x_size - 1));
    int y1 = (int) Math.floor((currentTranslation.getY() - miny) / (maxy - miny) * (grid_y_size - 1));
    int x2 = x1 + 1;
    int y2 = y1 + 1;
    
    double xfrac = (currentTranslation.getX() - minx) / (maxx - minx) * (grid_x_size - 1) - x1;
    double yfrac = (currentTranslation.getY() - miny) / (maxy - miny) * (grid_y_size - 1) - y1;
    
    LauncherState p1 = getGridPoint(x1, y1);
    LauncherState p2 = getGridPoint(x2, y1);
    LauncherState p3 = getGridPoint(x1, y2);
    LauncherState p4 = getGridPoint(x2, y2);
    
    double speed = (1 - xfrac) * (1 - yfrac) * p1.speed + xfrac * (1 - yfrac) * p2.speed + (1 - xfrac) * yfrac * p3.speed + xfrac * yfrac * p4.speed;
    double angle = (1 - xfrac) * (1 - yfrac) * p1.angleDegrees + xfrac * (1 - yfrac) * p2.angleDegrees + (1 - xfrac) * yfrac * p3.angleDegrees + xfrac * yfrac * p4.angleDegrees;
    
    return new LauncherState(speed, angle * 0.9);
  }

  private double RPMToLinarSpeed(double rpm) {
    double loss = 0.2; // 80% loss in speed
    return rpm / 60. * // Revoluions per second
      Math.PI * Constants.Launcher.wheelRadiusMeters * 2 * // Circumferences per second
      loss;
  }
  private double linearSpeedToRPM(double linarSpeed) {
    double loss = 0.2; // 80% loss in speed
    return linarSpeed / // Circumferences per second
      (Math.PI * Constants.Launcher.wheelRadiusMeters * 2) * // Revoluions per second
      60. / loss;
  }

  private LauncherState getLauncherStateMathematicalModel() {
    // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)
    // We find the "ideal" angle and speed by using the lowest speed that allows us to get to the target
    
    Swerve swerve = Swerve.getInstance();
    double lookaheadDistance = swerve.getRobotSpeed() * 0.2; // 0.2 is abritrary; let's say it takes 200ms to adjust the launcher
    Pose2d pose = swerve.getPose().transformBy(new Transform2d(lookaheadDistance, 0.0, new Rotation2d()));
    Translation2d currentTranslation = pose.getTranslation();

    double speakerInward = -0.1;
    boolean isBlueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    Translation2d allianceSpeakerTranslation = isBlueAlliance ? new Translation2d(speakerInward, 5.55) : new Translation2d(Constants.fieldLengthMeters - speakerInward, 5.55);

    // Project the position of the robot and speaker into a 2D plane, with the robot at (0, 0)
    double robotDistance = currentTranslation.getDistance(allianceSpeakerTranslation); // x
    double speakerHeight = 1.98; // y

    double g = 6.67; // Gravitational constant

    // Original equation:
    // Math.pow(speed, 4) - g*(g*robotDistance*robotDistance + 2*speakerHeight*speed*speed) = 0
    // Solving for for s in `s^4 - g*(g*d*d + 2*h*s*s) = 0` gives 4 equations, only one of which we want:
    // x = sqrt(sqrt(g^2 * (d^2 + h^2)) + g*h)
    // (Where x is the minimum speed)
    // Here's a graph: https://www.desmos.com/calculator/oxdtf5krk9

    double minimumSpeedMPS = Math.sqrt(
      Math.sqrt(g*g * (robotDistance*robotDistance + speakerHeight*speakerHeight)) + g*speakerHeight
    ) / (2 * Math.PI * Constants.Launcher.wheelRadiusMeters);

    double minimumSpeedRPM = linearSpeedToRPM(minimumSpeedMPS);
    double usedSpeedRPM = minimumSpeedRPM + 50;

    // Convert RPM to meters per second
    double speed = RPMToLinarSpeed(usedSpeedRPM);
    
    double angleRadians = Math.atan(
      (
        speed*speed +
        Math.sqrt(
          Math.pow(speed, 4) -
          g*(g*robotDistance*robotDistance + 2*speakerHeight*speed*speed)
        )
      ) /
      (g * robotDistance)
    );

    return new LauncherState(usedSpeedRPM, Units.radiansToDegrees(angleRadians));
  }

  // Higher shot time = more correction for velocity (and acceleration depending on pipeline latency)
  public static double getShotTime(double distance) {
    if(SwerveAlignmentController.getInstance().getAlignmentMode() == AlignmentMode.AllianceSpeaker) return 0.35 * distance + 0.05;
    else return 0.538 + -0.501 * distance + 0.218 * Math.pow(distance, 2) + -0.0147 * Math.pow(distance, 3);
  }

  // https://docs.google.com/spreadsheets/d/1dXLGZ84TEYzmvrYkXQ3SZOwmgObqmYSdxnnUZoMg85Bo/edit?usp=sharing
  // Very rudimentary model for now
  private static double getSpeed(double distance, Rotation2d angle) {
    double speakerAngle = angle.getDegrees();
    double startAngle = SwerveAlignmentController.farInterpolationStartAngle.getDegrees();
    double angleWidth = SwerveAlignmentController.farInterpolationWidth.getDegrees();
    if(speakerAngle < startAngle) {
      return getSpeedAcute(distance);
    } else if(speakerAngle > startAngle + angleWidth) {
      return getSpeedObtuse(distance);
    } else {
      double interpolation = (speakerAngle - startAngle) / angleWidth;
      double acute = getSpeedAcute(distance);
      double obtuse = getSpeedObtuse(distance);
      return MathUtil.interpolate(acute, obtuse, interpolation);
    }
  }
  private static double getAngle(double distance, Rotation2d angle) {
    double speakerAngle = angle.getDegrees();
    double startAngle = SwerveAlignmentController.farInterpolationStartAngle.getDegrees();
    double angleWidth = SwerveAlignmentController.farInterpolationWidth.getDegrees();
    if(speakerAngle < startAngle) {
      return getAngleAcute(distance);
    } else if(speakerAngle > startAngle + angleWidth) {
      return getAngleObtuse(distance);
    } else {
      double interpolation = (speakerAngle - startAngle) / angleWidth;
      double acute = getAngleAcute(distance);
      double obtuse = getAngleObtuse(distance);
      return MathUtil.interpolate(acute, obtuse, interpolation);
    }
  }

  private static double getSpeedAcute(double distance) {
    if(SwerveAlignmentController.getInstance().getAlignmentMode() == AlignmentMode.AllianceSpeaker) return 6125;
    else return getLobSpeed(distance);
  }
  private static double getAngleAcute(double distance) {
    if(SwerveAlignmentController.getInstance().getAlignmentMode() == AlignmentMode.AllianceSpeaker) return 97.2 * Math.exp(-0.473 * distance) * 1.06 + 0.8;
    else return getLobAngle(distance);
  }
  private static double getSpeedObtuse(double distance) {
    if(SwerveAlignmentController.getInstance().getAlignmentMode() == AlignmentMode.AllianceSpeaker) return 6125;
    else return getLobSpeed(distance);
  }
  private static double getAngleObtuse(double distance) {
    if(SwerveAlignmentController.getInstance().getAlignmentMode() == AlignmentMode.AllianceSpeaker) return getAngleAcute(distance) * 1.035 - 0.2;
    else return getLobAngle(distance);
  }

  private static double getLobSpeed(double distance) {
    distance = Math.min(distance, 10);
    return 756 + 185 * distance + 49.8 * Math.pow(distance, 2) + -3.92 * Math.pow(distance, 3);
  }
  private static double getLobAngle(double distance) {
    distance = Math.min(distance, 10);
    return 19.2 + 1.63 * distance + 1.17 * Math.pow(distance, 2) + -0.0969 * Math.pow(distance, 3);
  }

  private LauncherState getLauncherStateTimeBasedPrediction() {
    double distance = SwerveAlignmentController.getInstance().currentShotDistance;
    Rotation2d speakerAngle = SwerveAlignmentController.getInstance().targetAngle;
    Logger.recordOutput("Launcher/AutomaticControl/Distance", distance);
    Logger.recordOutput("Launcher/AutomaticControl/SpeakerAngle", speakerAngle);

    double speed = MathUtil.clamp(getSpeed(distance, speakerAngle), 150, 6800);
    double angle = MathUtil.clamp(getAngle(distance, speakerAngle), 20, 60);

    return new LauncherState(speed, angle);
  }
  
  public enum LauncherControlType {
    LookupTable,
    MathematicalModel,
    TimeBasedPrediction
  }

  /**
   * Automatically adjusts the launcher angle and speed based on the robot's position and a predefined lookup table.
   */
  public void autoAlign() {
    autoAlign(false);
  }

  /**
   * Automatically adjusts the launcher angle and speed based on the robot's position and a predefined lookup table.
   */
  public void autoAlign(boolean inAuto) {
    LauncherControlType controlType = LauncherControlType.TimeBasedPrediction;
    LauncherState state;
    switch(controlType) {
      case MathematicalModel:
        state = getLauncherStateMathematicalModel();
        break;
      case TimeBasedPrediction:
        state = getLauncherStateTimeBasedPrediction();
        break;
      case LookupTable:
      default:
        state = getLauncherStateLookupTable();
        break;
    }

    Logger.recordOutput("Launcher/AutomaticControl/State/Speed", state.speed);
    Logger.recordOutput("Launcher/AutomaticControl/State/Angle", state.angleDegrees);
    
    if(state.speed == 0) return;
    
    Launcher launcher = Launcher.getInstance();
    launcher.setLauncherState(state);
    
    boolean launcherAtSetpoints = launcher.atSetpoints();
    boolean atTarget = SwerveAlignmentController.getInstance().atTarget;
    Logger.recordOutput("Launcher/AutomaticControl/AtSetpoints", launcherAtSetpoints);
    Logger.recordOutput("Launcher/AutomaticControl/AtTarget", atTarget);
    if(!inAuto && launcherAtSetpoints && atTarget) {
      VibrationFeedback.getInstance().addToOperatorLeft(1.0);
      VibrationFeedback.getInstance().addToOperatorRight(1.0);
      VibrationFeedback.getInstance().addToDriverLeft(0.3);
      VibrationFeedback.getInstance().addToDriverRight(0.3);
    }
  }
}