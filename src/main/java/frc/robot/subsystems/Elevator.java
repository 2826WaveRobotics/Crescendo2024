package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    // Elevator Motor to extend and retract the Elevator
    private CANSparkMax elevatorExtRetMotor;

    // Elevator Motor to swing up and down the Elevator
    private CANSparkMax elevatorAngleMotor;

    // Saves location og the Elevator in the vertical direction
    private RelativeEncoder elevatorExtRetEncoder;

    // Saves location of the Elevator angle (normally swing from straight up to the Launcher position)
    private RelativeEncoder elevatorAngleEncoder;

    // Internal PID Controller for the Elevator extend-retract Motor
    private SparkPIDController elevatorExtRetPIDController;

    // Internal PID Controller for the Elevator angular position
    private SparkPIDController elevatorAnglePIDController;

    // Current command to move the elevator angular direction
    private double targetElevatorAngle;

    // Current commanded velocity to extend and retract the elevator
    private double targetElevatorPosition;

    /**
     * The through beam sensor for detecting if notes are in the intake.
     */
    private DigitalInput intakeSensor = new DigitalInput(Constants.Elevator.intakeSensorDIOPort);
    /**
     * The through beam sensor detecting if the note is in position.
     */
    private DigitalInput noteInPositionSensor = new DigitalInput(Constants.Elevator.noteInPositionSensorDIOPort);
    /**
     * The through beam sensor detecting if the note is transitioning to the resting position.
     */
    private DigitalInput noteInTransitionSensor = new DigitalInput(Constants.Elevator.noteInTransitionSensorDIOPort);

    /**
     * Gets if the through beam sensor for detecting if notes are in the intake is activated (meaning there's a note there).
     * @return
     */
    public boolean getIntakeSensorActivated() {
        return !intakeSensor.get();
    }

    /**
     * Gets if the through beam sensor for detecting if the note is in position is activated (meaning there's a note there).
     * @return
     */
    public boolean getNoteInPositionSensorActivated() {
        return !noteInPositionSensor.get();
    }

    /**
     * Gets if the through beam sensor for detecting if the note is transitioning to the resting position is activated
     * (meaning there's a note there).
     * @return
     */
    public boolean getNoteInTransitionSensorActivated() {
        return !noteInTransitionSensor.get();
    }

    public Elevator() {
        elevatorExtRetMotor = new CANSparkMax(Constants.Elevator.positionMotorCANID, MotorType.kBrushless);
        elevatorExtRetMotor.setInverted(false);
        elevatorAngleMotor = new CANSparkMax(Constants.Elevator.angleMotorCANID, MotorType.kBrushless);
        elevatorAngleMotor.setInverted(false);

        elevatorExtRetEncoder = elevatorExtRetMotor.getEncoder();
        elevatorAngleEncoder = elevatorAngleMotor.getEncoder();

        elevatorExtRetMotor.restoreFactoryDefaults();
        elevatorAngleMotor.restoreFactoryDefaults();

        elevatorExtRetPIDController = elevatorExtRetMotor.getPIDController();
        elevatorAnglePIDController = elevatorAngleMotor.getPIDController();

        configMotorControllers();

        // temporary for testing
        targetElevatorPosition = 0; //1000; 
        targetElevatorAngle = 0;

    }

    public void configMotorControllers() {
        elevatorExtRetMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(elevatorExtRetMotor, Usage.kPositionOnly);
        elevatorExtRetMotor.setSmartCurrentLimit(Constants.Elevator.extRetCurrentLimit);
        elevatorExtRetMotor.setIdleMode(Constants.Elevator.elevatorIdleMode);
        elevatorExtRetPIDController.setP(Constants.Elevator.elevatorKP);
        elevatorExtRetPIDController.setD(Constants.Elevator.elevatorKD);
        elevatorExtRetPIDController.setI(Constants.Elevator.elevatorKI);
        elevatorExtRetPIDController.setFF(Constants.Elevator.elevatorKFF);
        elevatorExtRetMotor.enableVoltageCompensation(Constants.Elevator.voltageComp);
        elevatorExtRetMotor.setInverted(Constants.Elevator.invertPosition);
        elevatorExtRetMotor.burnFlash();
           
        elevatorAngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(elevatorAngleMotor, Usage.kPositionOnly);
        elevatorAngleMotor.setSmartCurrentLimit(Constants.Elevator.eAngleCurrentLimit);
        elevatorAngleMotor.setIdleMode(Constants.Elevator.eAngleIdleMode);
        elevatorAnglePIDController.setP(Constants.Elevator.eAngleKP);
        elevatorAnglePIDController.setI(Constants.Elevator.eAngleKI);
        elevatorAnglePIDController.setD(Constants.Elevator.eAngleKD);
        elevatorAnglePIDController.setFF(Constants.Elevator.eAngleKFF);
        elevatorAngleMotor.enableVoltageCompensation(Constants.Elevator.voltageComp);
        elevatorAngleMotor.setInverted(Constants.Elevator.invertAngle);
        elevatorAngleMotor.burnFlash();
    }    
    
    @Override
    public void periodic() {
        // Elevator extend-retract command
        elevatorExtRetPIDController.setReference(targetElevatorPosition * Constants.Elevator.elevatorPositionGearboxRatio, CANSparkMax.ControlType.kVelocity);

        // Elevator angular movement command
        elevatorAnglePIDController.setReference(targetElevatorAngle, CANSparkMax.ControlType.kPosition);

    }

    /**
     * Runs the elevator upward until it 
     */
    public void extendElevator() {        
    }

    public void retractElevator() {
    }
}
