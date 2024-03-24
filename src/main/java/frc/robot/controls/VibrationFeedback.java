package frc.robot.controls;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
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

    private double driverOverrideLeft = 0;
    private double driverOverrideRight = 0;
    private double operatorOverrideLeft = 0;
    private double operatorOverrideRight = 0;
    private double currentLeftDriver = 0;
    private double currentRightDriver = 0;
    private double currentLeftOperator = 0;
    private double currentRightOperator = 0;
    public void setDriverOverrideLeft(double override) {
        driverOverrideLeft = override;
        Controls.getInstance().setDriverRumble(currentLeftDriver + override, currentRightDriver + driverOverrideRight);
    }
    public void setDriverOverrideRight(double override) {
        driverOverrideRight = override;
        Controls.getInstance().setDriverRumble(currentLeftDriver + driverOverrideLeft, currentRightDriver + override);
    }
    public void setOperatorOverrideLeft(double override) {
        operatorOverrideLeft = override;
        Controls.getInstance().setOperatorRumble(currentLeftOperator + override, currentRightOperator + operatorOverrideRight);
    }
    public void setOperatorOverrideRight(double override) {
        operatorOverrideRight = override;
        Controls.getInstance().setOperatorRumble(currentLeftOperator + operatorOverrideLeft, currentRightOperator + override);
    }

    private class SetVibrationCommand extends InstantCommand {
        private static void setVibration(Controller controller, double left, double right) {
            VibrationFeedback vibrationFeedback = VibrationFeedback.getInstance();
            switch (controller) {
                case Driver:
                    Controls.getInstance().setDriverRumble(
                        Math.min(left + vibrationFeedback.driverOverrideLeft, 1),
                        Math.min(right + vibrationFeedback.driverOverrideRight, 1)
                    );
                    vibrationFeedback.currentLeftDriver = left;
                    vibrationFeedback.currentRightDriver = right;
                    break;
                case Operator:
                    Controls.getInstance().setOperatorRumble(
                        Math.min(left + vibrationFeedback.operatorOverrideLeft, 1),
                        Math.min(right + vibrationFeedback.operatorOverrideRight, 1)
                    );
                    vibrationFeedback.currentLeftOperator = left;
                    vibrationFeedback.currentRightOperator = right;
                    break;
                case Both:
                    Controls.getInstance().setDriverRumble(
                        Math.min(left + vibrationFeedback.driverOverrideLeft, 1),
                        Math.min(right + vibrationFeedback.driverOverrideRight, 1)
                    );
                    vibrationFeedback.currentLeftDriver = left;
                    vibrationFeedback.currentRightDriver = right;
                    Controls.getInstance().setOperatorRumble(
                        Math.min(left + vibrationFeedback.operatorOverrideLeft, 1),
                        Math.min(right + vibrationFeedback.operatorOverrideRight, 1)
                    );
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
