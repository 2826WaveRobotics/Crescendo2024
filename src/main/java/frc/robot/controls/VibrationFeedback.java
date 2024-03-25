package frc.robot.controls;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class VibrationFeedback {
    private static VibrationFeedback instance = null;
    public static VibrationFeedback getInstance() {
        if (instance == null) {
            instance = new VibrationFeedback();
        }
        return instance;
    }

    private final Timer matchTimer = new Timer();

    public void teleopInit() {
        matchTimer.reset();
        matchTimer.start();

        reset();
    }
    public void teleopExit() {
        matchTimer.stop();
    }

    EventLoop eventLoop = new EventLoop();
    Notifier timeNotifier = new Notifier(() -> eventLoop.poll());
    
    private VibrationFeedback() {
        new Trigger(eventLoop, () -> matchTimer.hasElapsed(75)).onTrue(
            new InstantCommand(() -> runPattern(VibrationPatternType.SixtySecondWarning)));
        new Trigger(eventLoop, () -> matchTimer.hasElapsed(115)).onTrue(
            new InstantCommand(() -> runPattern(VibrationPatternType.TwentySecondWarning)));
        timeNotifier.startPeriodic(0.5);
    }

    private enum Controller {
        Driver,
        Operator,
        Both
    }

    private double driverAddLeft = 0;
    private double driverAddRight = 0;
    private double operatorAddLeft = 0;
    private double operatorAddRight = 0;

    private double currentLeftDriver = 0;
    private double currentRightDriver = 0;
    private double currentLeftOperator = 0;
    private double currentRightOperator = 0;

    /**
     * Resets all rumble values to 0.
     */
    public void reset() {
        driverAddLeft = 0;
        driverAddRight = 0;
        operatorAddLeft = 0;
        operatorAddRight = 0;

        currentLeftDriver = 0;
        currentRightDriver = 0;
        currentLeftOperator = 0;
        currentRightOperator = 0;
    }

    private EventLoop turnOffRumbleFailsafeEventLoop = new EventLoop();
    private BooleanEvent turnOffDriverRumbleFailsafe =
        new BooleanEvent(turnOffRumbleFailsafeEventLoop, () -> currentLeftDriver != 0 || currentRightDriver != 0)
        .debounce(5., DebounceType.kRising)
        .rising();
    private BooleanEvent turnOffOperatorRumbleFailsafe =
        new BooleanEvent(turnOffRumbleFailsafeEventLoop, () -> currentLeftOperator != 0 || currentRightOperator != 0)
        .debounce(5., DebounceType.kRising)
        .rising();

    /**
     * Updates the rumble values based on the current rumble values and the added rumble values.
     */
    public void update() {
        // If the driver or operator rumbles have been on for too long, something might have went wrong.
        // This isn't a critical error, but it would be incredibly annoying for the driver and operator
        // if their controllers were constantly rumbling.
        turnOffRumbleFailsafeEventLoop.poll();
        if(turnOffDriverRumbleFailsafe.getAsBoolean()) {
            currentLeftDriver = 0;
            currentRightDriver = 0;
        }
        if(turnOffOperatorRumbleFailsafe.getAsBoolean()) {
            currentLeftOperator = 0;
            currentRightOperator = 0;
        }

        // WPILib clamps rumble values to 0-1, so we don't need to do it here
        Controls.getInstance().setDriverRumble(currentLeftDriver + driverAddLeft,currentRightDriver + driverAddRight);
        Controls.getInstance().setOperatorRumble(currentLeftOperator + operatorAddLeft,currentRightOperator + operatorAddRight);
    }

    public void setDriverAddLeft(double override) {
        driverAddLeft = override;
    }
    public void setDriverAddRight(double override) {
        driverAddRight = override;
    }
    public void setOperatorAddLeft(double override) {
        operatorAddLeft = override;
    }
    public void setOperatorAddRight(double override) {
        operatorAddRight = override;
    }

    private class SetVibrationCommand extends InstantCommand {
        private static void setVibration(Controller controller, double left, double right) {
            VibrationFeedback vibrationFeedback = VibrationFeedback.getInstance();
            switch (controller) {
                case Driver:
                    vibrationFeedback.currentLeftDriver = left;
                    vibrationFeedback.currentRightDriver = right;
                    break;
                case Operator:
                    vibrationFeedback.currentLeftOperator = left;
                    vibrationFeedback.currentRightOperator = right;
                    break;
                case Both:
                    vibrationFeedback.currentLeftDriver = left;
                    vibrationFeedback.currentRightDriver = right;
                    vibrationFeedback.currentLeftOperator = left;
                    vibrationFeedback.currentRightOperator = right;
                    break;
            }
        }

        public SetVibrationCommand(Controller controller, double left, double right) {
            super(() -> setVibration(controller, left, right));
        }
    }

    private class VibrationPulse extends SequentialCommandGroup {
        public VibrationPulse(Controller controller, double left, double right, double duration, double wait) {
            addCommands(
                new SetVibrationCommand(controller, left, right),
                new WaitCommand(duration),
                new SetVibrationCommand(controller, 0, 0),
                new WaitCommand(wait)
            );
        }
        public VibrationPulse(Controller controller, double left, double right, double duration) {
            this(controller, left, right, duration, 0.05);
        }
        public VibrationPulse(Controller controller, double left, double right) {
            this(controller, left, right, 0.1);
        }
    }

    public enum VibrationPatternType {
        IntakingNote,
        ClimberSideDown,

        SixtySecondWarning,
        TwentySecondWarning
    }

    public void runPattern(VibrationPatternType pattern) {
        switch (pattern) {
            case IntakingNote:
                new SequentialCommandGroup(
                    new VibrationPulse(Controller.Operator, 0.75, 0.5),
                    new VibrationPulse(Controller.Operator, 0.5, 0.75),
                    new VibrationPulse(Controller.Operator, 0.5, 0.5)
                ).schedule();
                break;
            case ClimberSideDown:
                new SequentialCommandGroup(
                    new VibrationPulse(Controller.Both, 0.5, 0.5),
                    new VibrationPulse(Controller.Both, 0.5, 0.5)
                ).schedule();
                break;
            case SixtySecondWarning:
                new SequentialCommandGroup(
                    new VibrationPulse(Controller.Both, 0.5, 0.5, 0.3, 0.1),
                    new VibrationPulse(Controller.Both, 0.35, 0.35, 0.3, 0.1)
                ).schedule();
                break;
            case TwentySecondWarning:
                new VibrationPulse(Controller.Both, 0.5, 0.5, 0.3).schedule();
                break;
        }
    }
}
