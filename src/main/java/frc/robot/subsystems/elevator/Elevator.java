package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase implements AutoCloseable {
    private static Elevator instance = null;
    public static Elevator getInstance() {
        if (instance == null) {
            switch(Constants.currentMode) {
                case REAL:
                    instance = new Elevator(new ElevatorIOReal());
                    return instance;
                default:
                    instance = new Elevator(new ElevatorIO() {});
                    return instance;
            }
        }
        return instance;
    }

    /**
     * The states that the elevator can be in.
     */
    public enum ElevatorState {
        /**
         * The state where the elevator is contracted and angled downward (pointing toward the launcher).
         */
        Stowed,
        /**
         * The state where the elevator is contracted but angled upward (perpendicular to the robot base).
         */
        AngledUp, 
        /**
         * The state where the elevator is extended and angled upward.
         */
        Extended
    }

    public ElevatorState currentState = ElevatorState.Stowed;

    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    /**
     * Resets the encoder value for the angle motor to 0.
     */
    public void resetAngleEncoder() {
        elevatorIO.resetAngleEncoder();
    }
    
    /**
     * Resets the encoder value for the extension motor to 0.
     */
    public void resetExtensionEncoder() {
        elevatorIO.resetExtensionEncoder();
    }

    /**
     * Sets the extension motor speed in RPM. Positive numbers move the elevator downward.  
     * @param speed The target speed, in RPM.
     */
    public void setExtensionSpeed(double speed) {
        elevatorIO.setExtensionSpeed(speed);
    }

    /**
     * Sets the angle motor speed in RPM. Positive numbers rotate toward the intake side of the robot, which is the direction of the elevator pointing upward.
     * @param speed
     */
    public void setAngleSpeed(double speed) {
        elevatorIO.setAngleSpeed(speed);
    }

    /**
     * Sets the angle motor position, where 0 is the resting position and positive numbers rotate upward.
     * @param angle
     */
    public void setAnglePosition(Rotation2d angle) {
        elevatorIO.setAnglePosition(angle);
    }

    /**
     * Sets the extension position in meters, where 0 is fully contracted.
     * @param position
     */
    public void setExtensionPosition(double position) {
        elevatorIO.setExtensionPosition(position);
    }

    /**
     * Checks if the extension motor is currently stalling.
     * @return
     */
    public boolean extensionMotorIsStalling() {
        return inputs.extensionOutputCurrent > 10;
    }

    /**
     * Checks if the angle motor is currently stalling.
     * @return
     */
    public boolean angleMotorIsStalling() {
        return inputs.angleOutputCurrent > 10;
    }

    /**
     * Gets the current extension height of the elevator.
     * @return
     */
    public double getExtensionHeight() {
        return inputs.extensionHeight;
    }
    /**
     * Gets the absolute angle of the elevator, as measured by the absolute encoder.
     * @return
     */
    public Rotation2d getAbsoluteAngle() {
        return inputs.absoluteElevatorAngle;
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(inputs);
    }
    
    @Override
    public void close() {
        elevatorIO.close();
    }
}
