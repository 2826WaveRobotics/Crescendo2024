package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transport extends SubsystemBase {
    private CANSparkMax upperTransportMotor;
    private CANSparkMax lowerTransportMotor;

    private SparkPIDController upperTransportPIDController;
    private SparkPIDController lowerTransportPIDController;

    public Transport() {
        upperTransportMotor = new CANSparkMax(Constants.Transport.upperTransportMotorCANID, CANSparkMax.MotorType.kBrushless);
        lowerTransportMotor = new CANSparkMax(Constants.Transport.lowerTransportMotorCANID, CANSparkMax.MotorType.kBrushless);

        upperTransportPIDController = upperTransportMotor.getPIDController();
        lowerTransportPIDController = lowerTransportMotor.getPIDController();

        configMotorControllers();

    }

    private void configMotorControllers() {
        upperTransportMotor.restoreFactoryDefaults();
        upperTransportMotor.setSmartCurrentLimit(Constants.Transport.transportCurrentLimit);
        upperTransportMotor.enableVoltageCompensation(Constants.Transport.voltageComp);
        upperTransportMotor.setIdleMode(Constants.Transport.transportIdleMode);
        upperTransportPIDController.setP(Constants.Transport.upperTransportKP);
        upperTransportPIDController.setI(Constants.Transport.upperTransportKI);
        upperTransportPIDController.setD(Constants.Transport.upperTransportKD);
        upperTransportPIDController.setFF(Constants.Transport.upperTransportKFF);
        upperTransportMotor.burnFlash();

        lowerTransportMotor.restoreFactoryDefaults();
        lowerTransportMotor.setSmartCurrentLimit(Constants.Transport.transportCurrentLimit);
        lowerTransportMotor.enableVoltageCompensation(Constants.Transport.voltageComp);
        lowerTransportMotor.setIdleMode(Constants.Transport.transportIdleMode);
        lowerTransportPIDController.setP(Constants.Transport.lowerTransportKP);
        lowerTransportPIDController.setI(Constants.Transport.lowerTransportKI);
        lowerTransportPIDController.setD(Constants.Transport.lowerTransportKD);
        lowerTransportPIDController.setFF(Constants.Transport.lowerTransportKFF);
        lowerTransportMotor.burnFlash();
    }

    public void lowerTransportOn() {
        lowerTransportPIDController.setReference(-5500, ControlType.kVelocity);
    }

    public void lowerTransportOff() {
        lowerTransportPIDController.setReference(0, ControlType.kVelocity);
    }

    public void upperTransportOn() {
        // TODO - Automatically change direction to score trap or load shooter
        upperTransportPIDController.setReference(-5500, ControlType.kVelocity);
    }

    public void upperTransportOff() {
        // TODO - Automatically change direction to score trap or load shooter
        upperTransportPIDController.setReference(0, ControlType.kVelocity);
    }
}
