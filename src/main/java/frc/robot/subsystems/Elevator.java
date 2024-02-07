package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    // Elevator Motor to extend and retract the Elevator
    private CANSparkMax elevatorExtRetMotor;
    // Elevator Motor to swing up and down the Elevator
    private CANSparkMax elevatorAngleMotor;
    // Elevator MotorA to roll the node from Intake to Elevator
    private CANSparkMax elevatorIntakeMotorA;
    // Elevator MotorB to roll the node from Intake to Elevator
    private CANSparkMax elevatorIntakeMotorB;
    // Elevator Tip Roller Motor to score the node 
    private CANSparkMax elevatorTipRollerMotorA;
    // Elevator Tip Roller Motor to score the node 
    private CANSparkMax elevatorTipRollerMotorB;

    // Saves location og the Elevator in the vertical direction
    private RelativeEncoder elevatorExtRetEncoder;
    // Saves location of the Elevator angle (normally swing from straight up to the Launcher position)
    private RelativeEncoder elevatorAngleEncoder;

    // Internal PID Controller for the Elevator extend-retract Motor
    private SparkPIDController elevatorExtRetPID;
    // Internal PID Controller for the Elevator angular position
    private SparkPIDController elevatorAnglePID;

    // Ideally these will be constants until PIDs parameters are tuned
    public double m1_kP; 
    public double m1_kI; 
    public double m1_kD;
    public double m1_kIz; 
    public double m1_kFF;
    public double kMaxOutput; 
    public double kMinOutput; 
    public double maxRPM;

    public double kMaxVelocity;
    public double kMaxAcceleration;


    public Elevator() {
        //insert comments about naming scheme here
        //Ext = exterior?
        elevatorExtRetMotor = new CANSparkMax(10, MotorType.kBrushless);
        elevatorExtRetMotor.setInverted(false);
        elevatorAngleMotor = new CANSparkMax(11, MotorType.kBrushless);
        elevatorAngleMotor.setInverted(false);

        elevatorExtRetEncoder = elevatorExtRetMotor.getEncoder();
        elevatorAngleEncoder = elevatorAngleMotor.getEncoder();

        elevatorExtRetMotor.restoreFactoryDefaults();
        elevatorAngleMotor.restoreFactoryDefaults();

        elevatorExtRetPID = elevatorExtRetMotor.getPIDController();
        elevatorAnglePID = elevatorAngleMotor.getPIDController();

        // PID coeffiecients
        m1_kP = 6e-5; 
        m1_kI = 0;
        m1_kD = 0; 
        m1_kIz = 0; 
        m1_kFF = 0.000175;

        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        kMaxAcceleration = 868;
        kMaxVelocity = 5000;


        // set PID coefficients for Extend-Retract motor
        elevatorExtRetPID.setP(m1_kP);
        elevatorExtRetPID.setI(m1_kI);
        elevatorExtRetPID.setD(m1_kD);
        elevatorExtRetPID.setIZone(m1_kIz);
        elevatorExtRetPID.setFF(m1_kFF);
        elevatorExtRetPID.setOutputRange(kMinOutput, kMaxOutput);

        // can set PID values with a gain & a slot ID
        elevatorExtRetPID.setSmartMotionMaxAccel(kMaxAcceleration, 0);
        elevatorExtRetPID.setSmartMotionMaxVelocity(kMaxVelocity, 0);

        // set PID coefficients for Extend-Retract motor
        elevatorAnglePID.setP(m1_kP);
        elevatorAnglePID.setI(m1_kI);
        elevatorAnglePID.setD(m1_kD);
        elevatorAnglePID.setIZone(m1_kIz);
        elevatorAnglePID.setFF(m1_kFF);
        elevatorAnglePID.setOutputRange(kMinOutput, kMaxOutput);

        elevatorAnglePID.setSmartMotionMaxAccel(kMaxAcceleration, 0);
        elevatorAnglePID.setSmartMotionMaxVelocity(kMaxVelocity, 0);
    }
    
    @Override
    public void periodic() {
        // Overrrides default code provided by SubsystemBase
        

    }
    
    
    public void trapDeposit() {
        // Intake motor
    
        // elevator

    }
}
