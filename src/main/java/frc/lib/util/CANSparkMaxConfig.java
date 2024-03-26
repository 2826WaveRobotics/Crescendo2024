package frc.lib.util;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class CANSparkMaxConfig {
    private class PIDValues {
        public int slot;
        public double p;
        public double i;
        public double d;
        public double f;

        public PIDValues(int slot, double p, double i, double d, double f) {
            this.slot = slot;
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
        }
    }

    /**
     * The idle mode of the motor.
     */
    public CANSparkMax.IdleMode idleMode;
    /**
     * The smart current limit in Amps.
     * <p>The motor controller will reduce the controller voltage output to avoid surpassing this limit. This limit is
     * enabled by default and used for brushless only. This limit is highly recommended when using the NEO brushless motor.  
     * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes that could
     * be enough to cause damage to the motor and controller. This current limit provides a smarter strategy
     * to deal with high current draws and keep the motor and controller operating in a safe region.
     */
    public int smartCurrentLimit;
    /**
     * The secondary current limit in Amps.
     * <p>The motor controller will disable the output of the controller briefly if the current limit
     * is exceeded to reduce the current. This limit is a simplified 'on/off' controller.
     * <p>This limit is enabled by default but is set higher than the default Smart Current Limit.  
     * <p>The time the controller is off after the current limit is reached is determined by the parameter
     * limitCycles, which is the number of PWM cycles (20kHz). The recommended value is the default of 0
     * which is the minimum time and is part of a PWM cycle from when the over current is detected.  
     * This allows the controller to regulate the current close to the limit value.
     * <p>The total time is set by the equation `t = (50us - t0) + 50us * limitCycles`  
     * <p>`t` = total off time after over current  
     * <p>`t0` = time from the start of the PWM cycle until over current is detected
     */
    public double secondaryCurrentLimit;
    /**
     * The maximum rate at which the motor controller's output is allowed to change.
     */
    public double closedLoopRampRate;

    /**
     * The voltage that this motor will run at so we can compensate.
     */
    public double voltageCompensation;

    /**
     * The CAN bus usage of this motor.
     */
    public Usage usage;

    public ArrayList<PIDValues> pidConfigurations = new ArrayList<PIDValues>();

    public CANSparkMaxConfig(
        CANSparkMax.IdleMode idleMode,
        int smartCurrentLimit,
        double secondaryCurrentLimit,
        double closedLoopRampRate,
        double voltageCompensation,
        Usage usage
    ) {
        this.idleMode = idleMode;
        this.smartCurrentLimit = smartCurrentLimit;
        this.secondaryCurrentLimit = secondaryCurrentLimit;
        this.closedLoopRampRate = closedLoopRampRate;
        this.voltageCompensation = voltageCompensation;
        this.usage = usage;
    }

    /**
     * Adds a PID slot to this Spark Max's configuration.
     * @param slot
     * @param p
     * @param i
     * @param d
     * @param f
     */
    public CANSparkMaxConfig configurePIDSlot(int slot, double p, double i, double d, double f) {
        pidConfigurations.add(new PIDValues(slot, p, i, d, f));
        return this;
    }

    /**
     * Gets the PID controller for the given slot.  
     * This is useful for configuring simulation models.
     * @param slot
     * @return
     */
    public PIDController getPIDController(int slot) {
        PIDValues values = pidConfigurations.get(slot);
        return new PIDController(values.p, values.i, values.d, values.f);
    }

    /**
     * Configures the given Spark Max and Spark PID controller from this configuration and burns the settings to flash.
     * @param spark
     * @param pidController
     */
    public void configure(CANSparkMax spark, SparkPIDController pidController) {
        configure(spark, pidController, true);
    }

    /**
     * Configures the given Spark Max and Spark PID controller from this configuration.
     * @param spark
     * @param pidController
     * @param burnFlash If we should burn the new settings to flash.
     */
    public void configure(CANSparkMax spark, SparkPIDController pidController, boolean burnFlash) {
        spark.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(spark, usage);
        spark.setSmartCurrentLimit(smartCurrentLimit);
        spark.setSecondaryCurrentLimit(secondaryCurrentLimit);
        // spark.setClosedLoopRampRate(closedLoopRampRate);

        spark.setIdleMode(idleMode);

        for(PIDValues values : pidConfigurations) {
            pidController.setP(values.p, values.slot);
            pidController.setI(values.i, values.slot);
            pidController.setD(values.d, values.slot);
            pidController.setFF(values.f, values.slot);
        }

        spark.enableVoltageCompensation(voltageCompensation);
        
        if(burnFlash) spark.burnFlash();
    }
}
