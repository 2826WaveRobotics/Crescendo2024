package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO, AutoCloseable {
    /**
     * The elevator motor that extends and retracts the elevator
     */
    private CANSparkMax elevatorExtensionMotor;

    /**
     * The elevator motor that changes the angle of the elevator
     */
    private CANSparkMax elevatorAngleMotor;

    private RelativeEncoder elevatorExtensionEncoder;

    // Internal PID Controller for the Elevator extend-retract Motor
    private SparkPIDController elevatorExtensionPIDController;

    // Internal PID Controller for the Elevator angular position
    private SparkPIDController elevatorAnglePIDController;

    /**
     * The through-bore absolute encoder on the elevator axis.
     */
    private DutyCycleEncoder elevatorAngleAbsoluteEncoder;

    public ElevatorIOReal() {
        elevatorExtensionMotor = new CANSparkMax(Constants.Elevator.positionMotorCANID, MotorType.kBrushless);
        elevatorExtensionMotor.setInverted(false);
        elevatorAngleMotor = new CANSparkMax(Constants.Elevator.angleMotorCANID, MotorType.kBrushless);
        elevatorAngleMotor.setInverted(false);

        elevatorExtensionEncoder = elevatorExtensionMotor.getEncoder();

        elevatorExtensionMotor.restoreFactoryDefaults();
        elevatorAngleMotor.restoreFactoryDefaults();

        elevatorExtensionPIDController = elevatorExtensionMotor.getPIDController();
        elevatorAnglePIDController = elevatorAngleMotor.getPIDController();

        elevatorAngleAbsoluteEncoder = new DutyCycleEncoder(Constants.Elevator.elevatorAbsoluteEncoderDIOPort);

        Constants.Elevator.positionMotorConfig.configure(elevatorExtensionMotor, elevatorExtensionPIDController);
        Constants.Elevator.angleMotorConfig.configure(elevatorAngleMotor, elevatorAnglePIDController);

        Shuffleboard.getTab("Sensor readouts").addNumber("DIO voltage", RobotController::getVoltage5V);
        Shuffleboard.getTab("Sensor readouts").addBoolean("Intake sensor", this::getIntakeSensorActivated);
        Shuffleboard.getTab("Sensor readouts").addBoolean("Note in transition sensor", this::getNoteInTransitionSensorActivated);
        Shuffleboard.getTab("Sensor readouts").addBoolean("Note in position sensor", this::getNoteInPositionSensorActivated);
    }

    /**
     * Resets the extension motor encoder.
     */
    @Override
    public void resetExtensionEncoder() {
        elevatorExtensionEncoder.setPosition(0);
    }

    /**
     * Resets the elevator angle encoder.
     */
    @Override
    public void resetAngleEncoder() {
        elevatorAngleAbsoluteEncoder.reset();
    }
    
    /**
     * Sets the extension motor speed in RPM. Positive numbers move the elevator downward.  
     * @param speed The target speed, in RPM.
     */
    @Override
    public void setExtensionSpeed(double speed) {
        elevatorExtensionPIDController.setReference(speed, ControlType.kVelocity, 1);
    }

    /**
     * Sets the angle motor speed in RPM. Positive numbers rotate toward the intake side of the robot, which is the direction of the elevator pointing upward.
     * @param speed
     */
    @Override
    public void setAngleSpeed(double speed) {
        elevatorAnglePIDController.setReference(speed, ControlType.kVelocity, 1);
    }

    /**
     * Sets the angle motor position, where 0 is the resting position and positive numbers rotate upward.
     * @param angle
     */
    @Override
    public void setAnglePosition(Rotation2d angle) {
        elevatorAnglePIDController.setReference(angle.getRotations(), ControlType.kPosition, 0);
    }

    /**
     * Sets the extension position in meters, where 0 is fully contracted.
     * @param position
     */
    @Override
    public void setExtensionPosition(double position) {
        elevatorExtensionPIDController.setReference(
            position / Constants.Elevator.totalElevatorExtensionHeight * Constants.Elevator.rotationsForFullExtension,
            ControlType.kPosition, 0
        );
    }

    /**
     * Gets the extension height of the elevator, relative to when `resetExtensionEncoder` was last called.
     * @return
     */
    private double getExtensionHeight() {
        return elevatorExtensionEncoder.getPosition()
            / Constants.Elevator.rotationsForFullExtension
            * Constants.Elevator.totalElevatorExtensionHeight;
    }

    /**
     * Gets the elevator angle in degrees, relative to the last time `resetAngleEncoder` was called.
     * 0 degrees is parallel to the robot frame, and the angle increases as the elevator tilts upward (in the direction of the intake side of the robot).
     * @return
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(elevatorAngleAbsoluteEncoder.getAbsolutePosition() - elevatorAngleAbsoluteEncoder.getPositionOffset());
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.absoluteElevatorAngle = getAngle();
        inputs.extensionHeight = getExtensionHeight();

        inputs.angleOutputCurrent = elevatorAngleMotor.getOutputCurrent();
        inputs.extensionOutputCurrent = elevatorExtensionMotor.getOutputCurrent();
    }

    @Override
    public void close() throws Exception {
        // The motor has a close method so the elevatorExtensionEcoder do not need to be closed
        elevatorExtensionMotor.close();
        elevatorAngleMotor.close();
        // This one is a standalone encoder (not a relative encoder - not a motor encoder) so it needs to be closed independently
        elevatorAngleAbsoluteEncoder.close();
    }
}
