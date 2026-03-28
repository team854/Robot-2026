package frc.robot.commands.smart;

import static edu.wpi.first.units.Units.Amp;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.StateMachine;
import frc.robot.subsystems.spindexer.SpindexerSubsystem.SpindexerState;
import frc.robot.subsystems.turret.KickerSubsystem.KickerState;

public class SmartShootCommand extends Command {
    public enum SmartShootStatus {
        FORWARD,
        REVERSE
    }

    private final StateMachine<SmartShootStatus> shootStateMachine;
    private final Debouncer overCurrentDebouncer;

    public SmartShootCommand() {
        shootStateMachine = new StateMachine<SmartShootCommand.SmartShootStatus>(SmartShootStatus.FORWARD, null);
        overCurrentDebouncer = new Debouncer(1, Debouncer.DebounceType.kRising);
        
        addRequirements(RobotContainer.spindexerSubsystem, RobotContainer.kickerSubsystem);
    }

    @Override
    public void initialize() {
        shootStateMachine.transitionTo(SmartShootStatus.FORWARD);
    }

    @Override
    public void execute() {
        switch (shootStateMachine.getCurrentState()) {
            case FORWARD:
                if (shootStateMachine.getStateTimer() > 0.5) {
                    if (overCurrentDebouncer.calculate(
                        RobotContainer.spindexerSubsystem.getMotorCurrent().in(Amp) >= Constants.SpindexerConstants.SPINDEXER_STALL_CURRENT.in(Amp)
                        || RobotContainer.kickerSubsystem.getMotorCurrent().in(Amp) >= Constants.KickerConstants.KICKER_STALL_CURRENT.in(Amp))) {
                        shootStateMachine.transitionTo(SmartShootStatus.REVERSE);
                    }
                }
                break;
            case REVERSE:
                if (shootStateMachine.getStateTimer() > 1) {
                    shootStateMachine.transitionTo(SmartShootStatus.FORWARD);
                }
                break;
        }

        switch (shootStateMachine.getCurrentState()) {
            case FORWARD:
                RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.READY, 5);
                RobotContainer.kickerSubsystem.requestDesiredState(KickerState.READY, 5);
                break;
            case REVERSE:
                RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.READY_REVERSE, 5);
                RobotContainer.kickerSubsystem.requestDesiredState(KickerState.READY_REVERSE, 5);
                break;
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.IDLE, 5);
        RobotContainer.kickerSubsystem.requestDesiredState(KickerState.IDLE, 5);
    }
}
