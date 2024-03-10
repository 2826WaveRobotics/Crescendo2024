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

    private class SetVibrationCommand extends InstantCommand {
        private static void setVibration(Controller controller, double left, double right) {
            switch (controller) {
                case Driver:
                    Controls.getInstance().setDriverRumble(left, right);
                    break;
                case Operator:
                    Controls.getInstance().setOperatorRumble(left, right);
                    break;
                case Both:
                    Controls.getInstance().setDriverRumble(left, right);
                    Controls.getInstance().setOperatorRumble(left, right);
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

        SixtySecondWarning,
        TwentySecondWarning
    }

    public void runPattern(VibrationPatternType pattern) {
        switch (pattern) {
            // case IntakingNote:
            //     new SequentialCommandGroup(
            //         new VibrationPulse(Controller.Operator, 0.75, 0.5),
            //         new VibrationPulse(Controller.Operator, 0.5, 0.75),
            //         new VibrationPulse(Controller.Operator, 0.5, 0.5)
            //     ).schedule();
            //     break;
            
            // case SixtySecondWarning:
            //     new SequentialCommandGroup(
            //         new VibrationPulse(Controller.Both, 0.5, 0.5, 0.3, 0.1),
            //         new VibrationPulse(Controller.Both, 0.35, 0.35, 0.3, 0.1)
            //     ).schedule();
            //     break;
            // case TwentySecondWarning:
            //     new VibrationPulse(Controller.Both, 0.5, 0.5, 0.3).schedule();
            //     break;
        }
    }
}
