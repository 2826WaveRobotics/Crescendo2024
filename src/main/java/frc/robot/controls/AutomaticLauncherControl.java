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

  private LauncherState getLauncherState() {
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