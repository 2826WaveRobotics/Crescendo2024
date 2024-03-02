package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private static Elevator instance = null;
    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
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

    /**
     * The elevator motor that extends and retracts the elevator
     */
    private CANSparkMax elevatorExtensionMotor;

    /**
     * The elevator motor that changes the angle of the elevator
     */
    private CANSparkMax elevatorAngleMotor;

    private RelativeEncoder elevatorExtensionEncoder;

    private RelativeEncoder elevatorAngleEncoder;

    // Internal PID Controller for the Elevator extend-retract Motor
    private SparkPIDController elevatorExtensionPIDController;

    // Internal PID Controller for the Elevator angular position
    private SparkPIDController elevatorAnglePIDController;

    /**
     * The through-bore absolute encoder on the elevator axis.
     */
    private DutyCycleEncoder elevatorAngleAbsoluteEncoder;

    private Elevator() {
        elevatorExtensionMotor = new CANSparkMax(Constants.Elevator.positionMotorCANID, MotorType.kBrushless);
        elevatorExtensionMotor.setInverted(false);
        elevatorAngleMotor = new CANSparkMax(Constants.Elevator.angleMotorCANID, MotorType.kBrushless);
        elevatorAngleMotor.setInverted(false);

        elevatorExtensionEncoder = elevatorExtensionMotor.getEncoder();
        elevatorAngleEncoder = elevatorAngleMotor.getEncoder();

        elevatorExtensionMotor.restoreFactoryDefaults();
        elevatorAngleMotor.restoreFactoryDefaults();

        elevatorExtensionPIDController = elevatorExtensionMotor.getPIDController();
        elevatorAnglePIDController = elevatorAngleMotor.getPIDController();

        elevatorAngleAbsoluteEncoder = new DutyCycleEncoder(Constants.Elevator.elevatorAbsoluteEncoderDIOPort);

        Constants.Elevator.positionMotorConfig.configure(elevatorExtensionMotor, elevatorExtensionPIDController);
        Constants.Elevator.angleMotorConfig.configure(elevatorAngleMotor, elevatorAnglePIDController);
    }

    /**
     * Resets the extension motor encoder.
     */
    public void resetExtensionEncoder() {
        elevatorExtensionEncoder.setPosition(0);
    }

    /**
     * Resets the elevator angle encoder.
     */
    public void resetAngleEncoder() {
        elevatorAngleAbsoluteEncoder.reset();
    }

    /**
     * Gets the extension height of the elevator, relative to when `resetExtensionEncoder` was last called.
     * @return
     */
    public double getExtensionHeight() {
        return elevatorExtensionEncoder.getPosition()
            / Constants.Elevator.rotationsForFullExtension
            * Constants.Elevator.totalElevatorExtensionHeight;
    }

    /**
     * Sets the extension motor speed in RPM. Positive numbers move the elevator downward.  
     * // TODO: We should probably use a PID based on position.
     * @param speed The target speed, in RPM.
     */
    public void setExtensionSpeed(double speed) {
        elevatorExtensionPIDController.setReference(speed, ControlType.kVelocity);
    }

    /**
     * Sets the angle motor speed in RPM. Positive numbers rotate toward the intake side of the robot, which is the direction of the elevator pointing upward.
     * @param speed
     */
    public void setAngleSpeed(double speed) {
        elevatorAnglePIDController.setReference(speed, ControlType.kVelocity);
    }

    /**
     * Checks if the extension motor is currently stalling.
     * @return
     */
    public boolean extensionMotorIsStalling() {
        // TODO: Figure out the real number.
        return elevatorExtensionMotor.getOutputCurrent() > 10;
    }

    /**
     * Checks if the angle motor is currently stalling.
     * @return
     */
    public boolean angleMotorIsStalling() {
        // TODO: Figure out the real number.
        return elevatorAngleMotor.getOutputCurrent() > 10;
    }

    /**
     * Gets the elevator angle in degrees, relative to the last time `resetAngleEncoder` was called.
     * 0 degrees is parallel to the robot frame, and the angle increases as the elevator tilts upward (in the direction of the intake side of the robot).
     * @return
     */
    public double getAngle() {
        return (elevatorAngleAbsoluteEncoder.getAbsolutePosition() - elevatorAngleAbsoluteEncoder.getPositionOffset()) * 360;
    }
}
