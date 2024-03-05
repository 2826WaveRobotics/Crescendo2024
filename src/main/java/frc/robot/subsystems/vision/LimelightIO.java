package frc.robot.subsystems.vision;

import java.io.ByteArrayOutputStream;
import java.io.ObjectOutputStream;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.LimelightHelpers;

public interface LimelightIO {
  public static class LimelightIOInputs {
    public LimelightHelpers.PoseEstimate poseEstimateData = null;
  }

  public static class SerializablePoseEstimate implements java.io.Serializable {
    public LimelightHelpers.PoseEstimate poseEstimate;
    SerializablePoseEstimate(LimelightHelpers.PoseEstimate poseEstimate) {
      this.poseEstimate = poseEstimate;
    }

    private void writeObject(ObjectOutputStream out) throws Exception {
      out.writeObject(poseEstimate);
    }
    private void readObject(java.io.ObjectInputStream in) throws Exception {
      poseEstimate = (LimelightHelpers.PoseEstimate) in.readObject();
    }
  }

  public class LimelightIOInputsLogged extends LimelightIO.LimelightIOInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
      try (ByteArrayOutputStream bos = new ByteArrayOutputStream()) {
        ObjectOutputStream oos = new ObjectOutputStream(bos);
        oos.writeObject(new SerializablePoseEstimate(poseEstimateData));
        table.put("PoseEstimateData", bos.toByteArray());
      } catch (java.io.IOException e) {
        System.out.println("Error writing LimelightIOInputsLogged to log: " + e);
      }
    }
  
    @Override
    public void fromLog(LogTable table) {
      try {
        byte[] data = table.get("PoseEstimateData", new byte[0]);
        if (data.length > 0) {
          java.io.ObjectInputStream ois = new java.io.ObjectInputStream(new java.io.ByteArrayInputStream(data));
          SerializablePoseEstimate poseEstimate = (SerializablePoseEstimate) ois.readObject();
          poseEstimateData = poseEstimate.poseEstimate;
        }
      } catch (java.io.IOException | ClassNotFoundException e) {
        System.out.println("Error reading LimelightIOInputsLogged from log: " + e);
      }
    }
  
    public LimelightIOInputsLogged clone() {
      LimelightIOInputsLogged copy = new LimelightIOInputsLogged();
      copy.poseEstimateData = this.poseEstimateData;
      return copy;
    }
  }
  

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LimelightIOInputs inputs) {}
}