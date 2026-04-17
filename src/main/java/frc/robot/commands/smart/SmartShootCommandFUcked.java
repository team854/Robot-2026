package frc.robot.commands.smart;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.StateMachine;
import frc.robot.subsystems.spindexer.SpindexerSubsystem.SpindexerState;
import frc.robot.subsystems.turret.KickerSubsystem.KickerState;
import frc.robot.subsystems.turret.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class SmartShootCommandFUcked extends Command {
    public enum SmartShootStatus {
        FORWARD,
        REVERSE
    }

    private final StateMachine<SmartShootStatus> shootStateMachine;
    private final Debouncer overCurrentDebouncer;

    public SmartShootCommandFUcked() {
        shootStateMachine = new StateMachine<SmartShootCommandFUcked.SmartShootStatus>(SmartShootStatus.FORWARD, null);
        overCurrentDebouncer = new Debouncer(1, Debouncer.DebounceType.kRising);
        
        addRequirements(RobotContainer.spindexerSubsystem, RobotContainer.kickerSubsystem);
    }

    @Override
    public void initialize() {
        
        shootStateMachine.transitionTo(SmartShootStatus.FORWARD);
    }

    @Override
    public void execute() {
        RobotContainer.shooterSubsystem.requestDesiredState(ShooterState.READY, 5);
        
        switch (shootStateMachine.getCurrentState()) {
            case FORWARD:
                if (shootStateMachine.getStateTimer() > 0.5) {
                    if (overCurrentDebouncer.calculate(
                        RobotContainer.spindexerSubsystem.getMotorCurrent().in(Amp) >= Constants.SpindexerConstants.SPINDEXER_REVERSE_CURRENT.in(Amp)
                        || RobotContainer.kickerSubsystem.getMotorCurrent().in(Amp) >= Constants.KickerConstants.KICKER_REVERSE_CURRENT.in(Amp))) {
                        shootStateMachine.transitionTo(SmartShootStatus.REVERSE);
                    }
                }
                break;
            case REVERSE:
                if (shootStateMachine.getStateTimer() > 0.5) {
                    shootStateMachine.transitionTo(SmartShootStatus.FORWARD);
                }
                break;
        }

        RobotContainer.turretSubsystem.requestDesiredState(TurretState.READY, 7);

        boolean shootingAllowed = true;

        if (RobotContainer.turretSubsystem.getCurrentState() != TurretState.READY) {
            shootingAllowed = false;
        } else if (RobotContainer.turretSubsystem.getTurretYaw().in(Radian) < Constants.TurretConstants.TURRET_YAW_LOWER_LIMIT.in(Radian) + Constants.ShooterConstants.SHOOTER_YAW_DEADZONE.in(Radian)
            || RobotContainer.turretSubsystem.getTurretYaw().in(Radian) > Constants.TurretConstants.TURRET_YAW_UPPER_LIMIT.in(Radian) - Constants.ShooterConstants.SHOOTER_YAW_DEADZONE.in(Radian)) {
            shootingAllowed = false;
        } else if (RobotContainer.shooterSubsystem.getCurrentState() != ShooterState.READY) {
            shootingAllowed = false;
        }

        

        if (shootingAllowed == false) {
            RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.IDLE, 7);
            RobotContainer.kickerSubsystem.requestDesiredState(KickerState.IDLE, 7);
            return;
        }

        switch (shootStateMachine.getCurrentState()) {
            case FORWARD:
                RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.READY, 7);
                RobotContainer.kickerSubsystem.requestDesiredState(KickerState.READY, 7);
                break;
            case REVERSE:
                RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.READY_REVERSE, 7);
                RobotContainer.kickerSubsystem.requestDesiredState(KickerState.READY, 7);
                break;
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.STOWED, 7);
        RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.IDLE, 7);
        RobotContainer.kickerSubsystem.requestDesiredState(KickerState.IDLE, 7);
    }
}
