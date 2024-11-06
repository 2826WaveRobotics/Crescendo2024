package frc.robot.controls;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/// An imperfect replica of the CommandXboxController class that uses NetworkTables to emulate button presses.
/// We only extend from CommandXboxController to make usage easier. There may be a more proper way to do this.
public class NTEmulatedCommandXboxController extends CommandXboxController {
    HashMap<String, LoggedDashboardBoolean> buttons = new HashMap<>();
    String name;

    public NTEmulatedCommandXboxController() {
        this("Default");
    }

    public NTEmulatedCommandXboxController(String name) {
        super(0);
        this.name = name;
    }

    public Trigger getButton(EventLoop loop, String button) {
        return new Trigger(loop, this.getLoggedButtonByName(button)::get);
    }

    public boolean getButtonValue(String button) {
        return this.getLoggedButtonByName(button).get();
    }

    private LoggedDashboardBoolean getLoggedButtonByName(String button) {
        return this.buttons.computeIfAbsent(button,
                k -> new LoggedDashboardBoolean("NTControls/" + name + "/" + button));
    }

    @Override
    public Trigger leftBumper() {
        return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger leftBumper(EventLoop loop) {
        return getButton(loop, "LEFT_BUMPER");
    }

    @Override
    public Trigger rightBumper() {
        return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightBumper(EventLoop loop) {
        return getButton(loop, "RIGHT_BUMPER");
    }

    @Override
    public Trigger leftStick() {
        return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger leftStick(EventLoop loop) {
        return getButton(loop, "LEFT_STICK_BUTTON");
    }

    @Override
    public Trigger rightStick() {
        return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightStick(EventLoop loop) {
        return getButton(loop, "RIGHT_STICK_BUTTON");
    }

    @Override
    public Trigger a() {
        return a(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger a(EventLoop loop) {
        return getButton(loop, "A");
    }

    @Override
    public Trigger b() {
        return b(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger b(EventLoop loop) {
        return getButton(loop, "B");
    }

    @Override
    public Trigger x() {
        return x(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger x(EventLoop loop) {
        return getButton(loop, "X");
    }

    @Override
    public Trigger y() {
        return y(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger y(EventLoop loop) {
        return getButton(loop, "Y");
    }

    @Override
    public Trigger start() {
        return start(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger start(EventLoop loop) {
        return getButton(loop, "START");
    }

    @Override
    public Trigger back() {
        return back(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger back(EventLoop loop) {
        return getButton(loop, "BACK");
    }

    @Override
    public Trigger leftTrigger() {
        return leftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    private Trigger leftTrigger(EventLoop loop) {
        return getButton(loop, "LEFT_TRIGGER");
    }

    @Override
    public Trigger leftTrigger(double threshold) {
        return leftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    } // Compatibility

    @Override
    public Trigger leftTrigger(double threshold, EventLoop loop) {
        return leftTrigger(loop);
    } // Compatibility

    @Override
    public Trigger rightTrigger() {
        return rightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    private Trigger rightTrigger(EventLoop loop) {
        return getButton(loop, "RIGHT_TRIGGER");
    }

    @Override
    public Trigger rightTrigger(double threshold) {
        return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    } // Compatibility

    @Override
    public Trigger rightTrigger(double threshold, EventLoop loop) {
        return rightTrigger(loop);
    } // Compatibility

    // TODO: non-boolean stick values? I'm not sure what the best way to handle this
    // is.
    @Override
    public double getLeftX() {
        return (getButtonValue("LEFT_X_NEG") ? -0.1 : 0) + (getButtonValue("LEFT_X_POS") ? 0.1 : 0);
    }

    @Override
    public double getRightX() {
        return (getButtonValue("RIGHT_X_NEG") ? -0.1 : 0) + (getButtonValue("RIGHT_X_POS") ? 0.1 : 0);
    }

    @Override
    public double getLeftY() {
        return (getButtonValue("LEFT_Y_NEG") ? -0.1 : 0) + (getButtonValue("LEFT_Y_POS") ? 0.1 : 0);
    }

    @Override
    public double getRightY() {
        return (getButtonValue("RIGHT_Y_NEG") ? -0.1 : 0) + (getButtonValue("RIGHT_Y_POS") ? 0.1 : 0);
    }

    @Override
    public double getLeftTriggerAxis() {
        return getButtonValue("LEFT_TRIGGER") ? 1 : 0;
    }

    @Override
    public double getRightTriggerAxis() {
        return getButtonValue("RIGHT_TRIGGER") ? 1 : 0;
    }
}
