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
    HashMap<String, LoggedDashboardBoolean> loggedButtons = new HashMap<>();
    String name;

    public NTEmulatedCommandXboxController() {
        this("Default");
    }
    public NTEmulatedCommandXboxController(String name) {
        super(0);
        this.name = name;
    }

    private Trigger getButtonByName(EventLoop loop, String button) {
        return new Trigger(
            loop,
            this.loggedButtons.computeIfAbsent(button, k -> new LoggedDashboardBoolean("NTControls/" + name + "/" + button))::get
        );
    }
    private boolean getButtonValueByName(String button) {
        return this.loggedButtons.computeIfAbsent(button, k -> new LoggedDashboardBoolean("NTControls/" + name + "/" + button)).get();
    }

    public Trigger leftBumper() { return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger leftBumper(EventLoop loop) { return getButtonByName(loop, "LEFT_BUMPER"); }

    public Trigger rightBumper() { return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger rightBumper(EventLoop loop) { return getButtonByName(loop, "RIGHT_BUMPER"); }

    public Trigger leftStick() { return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger leftStick(EventLoop loop) { return getButtonByName(loop, "LEFT_STICK_BUTTON"); }

    public Trigger rightStick() { return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger rightStick(EventLoop loop) { return getButtonByName(loop, "RIGHT_STICK_BUTTON"); }

    public Trigger a() { return a(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger a(EventLoop loop) { return getButtonByName(loop, "A"); }

    public Trigger b() { return b(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger b(EventLoop loop) { return getButtonByName(loop, "B"); }

    public Trigger x() { return x(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger x(EventLoop loop) { return getButtonByName(loop, "X"); }

    public Trigger y() { return y(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger y(EventLoop loop) { return getButtonByName(loop, "Y"); }

    public Trigger start() { return start(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger start(EventLoop loop) { return getButtonByName(loop, "START"); }

    public Trigger back() { return back(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger back(EventLoop loop) { return getButtonByName(loop, "BACK"); }

    public Trigger leftTrigger() { return leftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger leftTrigger(EventLoop loop) { return getButtonByName(loop, "LEFT_TRIGGER"); }
    public Trigger leftTrigger(double threshold) { return leftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop(), threshold); } // Compatibility
    public Trigger leftTrigger(EventLoop loop, double threshold) { return getButtonByName(loop, "LEFT_TRIGGER"); } // Compatibility

    public Trigger rightTrigger() { return rightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop()); }
    public Trigger rightTrigger(EventLoop loop) { return getButtonByName(loop, "RIGHT_TRIGGER"); }
    public Trigger rightTrigger(double threshold) { return rightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop(), threshold); } // Compatibility
    public Trigger rightTrigger(EventLoop loop, double threshold) { return getButtonByName(loop, "RIGHT_TRIGGER"); } // Compatibility

    public double getLeftX() { return (getButtonValueByName("LEFT_X_NEG") ? -1 : 0) + (getButtonValueByName("LEFT_X_POS") ? 1 : 0); }
    public double getRightX() { return (getButtonValueByName("RIGHT_X_NEG") ? -1 : 0) + (getButtonValueByName("RIGHT_X_POS") ? 1 : 0); }
    public double getLeftY() { return (getButtonValueByName("LEFT_Y_NEG") ? -1 : 0) + (getButtonValueByName("LEFT_Y_POS") ? 1 : 0); }
    public double getRightY() { return (getButtonValueByName("RIGHT_Y_NEG") ? -1 : 0) + (getButtonValueByName("RIGHT_Y_POS") ? 1 : 0); }

    public double getLeftTriggerAxis() { return getButtonValueByName("LEFT_TRIGGER") ? 1 : 0; }
    public double getRightTriggerAxis() { return getButtonValueByName("RIGHT_TRIGGER") ? 1 : 0; }
}
