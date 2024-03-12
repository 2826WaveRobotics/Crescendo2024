package frc.robot.controls;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.launcher.Launcher;

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

  private class LauncherState {
      public double speed;
      public double angleDegrees;
      public LauncherState(double speed, double angleDegrees) {
          this.speed = speed;
          this.angleDegrees = angleDegrees;
      }
  }

  private class LauncherLookupValue {
      public LauncherState state;
      public double x;
      public double y;

      public LauncherLookupValue(double x, double y, LauncherState state) {
          this.state = state;
          this.x = x;
          this.y = y;
      }
  }

  private ArrayList<LauncherLookupValue> lookupTable = new ArrayList<>();
  {
    try (BufferedReader br =
      new BufferedReader(new FileReader(new File(Filesystem.getDeployDirectory(), "launcherData.csv")))
    ) {
      // The first line is CSV labels
      @SuppressWarnings("unused")
      String labels = br.readLine();

      String line;
      while ((line = br.readLine()) != null) {
        String[] parts = line.split(",");
        lookupTable.add(new LauncherLookupValue(
          Float.parseFloat(parts[0]),
          Float.parseFloat(parts[1]),
          new LauncherState(
            Float.parseFloat(parts[2]),
            Float.parseFloat(parts[3])
          )
        ));
      }
    } catch(IOException ioException) {
      System.out.println("WARNING: IO exception while reading automatic launcher control file: " + ioException.getLocalizedMessage());
    }
  }

  private LauncherState getLauncherState() {
    Swerve swerve = Swerve.getInstance();

    // Find the closest 4 points in the lookup table and interpolate between them
    // Adjust for the robot's speed since by the time we reach the target, the robot will have moved
    double lookaheadDistance = swerve.getRobotSpeed() * 0.2; // 0.2 is abritrary; let's say it takes 200ms to adjust the launcher
    Pose2d pose = swerve.getPose().transformBy(new Transform2d(lookaheadDistance, 0.0, new Rotation2d()));
    Translation2d currentTranslation = pose.getTranslation();

    // If on the red alliance, flip the x coordinate
    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        currentTranslation = new Translation2d(-currentTranslation.getX(), currentTranslation.getY());
    }
    
    LauncherLookupValue closestPoints[] = new LauncherLookupValue[4];
    for (int i = 0; i < 4; i++) {
        closestPoints[i] = lookupTable.get(0);
    }
    double closestDistances[] = new double[4];
    for (int i = 0; i < 4; i++) {
        closestDistances[i] = Double.MAX_VALUE;
    }

    for (LauncherLookupValue value : lookupTable) {
        double distance = currentTranslation.getDistance(new Translation2d(value.x, value.y));
        for (int i = 0; i < 4; i++) {
            if (distance < closestDistances[i]) {
                closestDistances[i] = distance;
                closestPoints[i] = value;
                break;
            }
        }
    }

    // Interpolate between the 4 closest points using bilinear interpolation

    // interpolated value = (1 - alpha) * ((1 - beta) * p1 + beta * p2) + alpha * ((1 - beta) * p3 + beta * p4)
    // Source: https://stackoverflow.com/questions/23920976/bilinear-interpolation-with-non-aligned-input-points

    double alpha = (currentTranslation.getX() - closestPoints[0].x) / (closestPoints[1].x - closestPoints[0].x);
    double beta = (currentTranslation.getY() - closestPoints[0].y) / (closestPoints[2].y - closestPoints[0].y);

    LauncherState p1 = closestPoints[0].state;
    LauncherState p2 = closestPoints[1].state;
    LauncherState p3 = closestPoints[2].state;
    LauncherState p4 = closestPoints[3].state;

    double speed = (1 - alpha) * ((1 - beta) * p1.speed + beta * p2.speed) + alpha * ((1 - beta) * p3.speed + beta * p4.speed);
    double angleDegrees = (1 - alpha) * ((1 - beta) * p1.angleDegrees + beta * p2.angleDegrees) + alpha * ((1 - beta) * p3.angleDegrees + beta * p4.angleDegrees);

    return new LauncherState(speed, angleDegrees);
  }

  /**
   * Automatically adjusts the launcher angle and speed based on the robot's position and a predefined lookup table.
   */
  public void autoAlign() {
    LauncherState state = getLauncherState();

    // SmartDashboard.putString("Automatic launcher control value", "[" + state.speed + ", " + state.angleDegrees + "]");

    Launcher.getInstance().setLauncherSpeed(state.speed, true);
    Launcher.getInstance().setLauncherAngle(Rotation2d.fromDegrees(state.angleDegrees));
  }
}