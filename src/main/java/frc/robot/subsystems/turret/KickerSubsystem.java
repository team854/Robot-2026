package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ErrorConstants;
import frc.robot.RobotContainer;
import frc.robot.libraries.SubsystemStateMachine;
import frc.robot.subsystems.turret.CalculationSubsystem.Zone;

public class KickerSubsystem extends SubsystemStateMachine<frc.robot.subsystems.turret.KickerSubsystem.KickerState> {

    public enum KickerState {
        IDLE,
        STOWED,
        READY_REVERSE,
        READY,
    }
    
    private final KickerIO io;

    private double lastErrorTimestamp = Double.NEGATIVE_INFINITY;

    public KickerSubsystem(KickerIO io) {
        super(KickerState.IDLE, KickerState.IDLE);

        if (io == null) {
            throw new IllegalArgumentException("KickerIO cannot be null");
        }

        this.io = io;
    }

    public Current getMotorCurrent() {
        return Amp.of(io.getMotorCurrent());
    }

    public void checkCanHealth() {
        double timestamp = Timer.getFPGATimestamp();
        if (io.checkCANError()) {
            lastErrorTimestamp = timestamp;
        }

        if ((timestamp - lastErrorTimestamp) < Constants.HealthConstants.CAN_ERROR_PERSIST.in(Second)) {
            RobotContainer.healthSubsystem.reportError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        } else {
            RobotContainer.healthSubsystem.clearError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        }
    }

    @Override
    public void statePeriodicBefore() {
        if (RobotContainer.calculationSubsystem.getZone() == Zone.TRENCH) {
            requestDesiredState(KickerState.STOWED, 30);
        } else {
            requestDesiredState(KickerState.IDLE, 0); 
        }
    }

    @Override
    public void statePeriodic() {
        
        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == KickerState.STOWED) {
                    transitionTo(KickerState.STOWED);
                } else if (getDesiredState() == KickerState.READY_REVERSE) {
                    transitionTo(KickerState.READY_REVERSE);
                } else if (getDesiredState() == KickerState.READY) {
                    transitionTo(KickerState.READY);
                }
                break;
            case STOWED:
                if (getDesiredState() == KickerState.READY) {
                    transitionTo(KickerState.READY);
                } else if (getDesiredState() == KickerState.READY_REVERSE) {
                    transitionTo(KickerState.READY_REVERSE);
                } else if (getDesiredState() == KickerState.IDLE) {
                    transitionTo(KickerState.IDLE);
                }
                break;

            case READY_REVERSE:
                if (getDesiredState() == KickerState.STOWED) {
                    transitionTo(KickerState.STOWED);
                } else if (getDesiredState() == KickerState.READY) {
                    transitionTo(KickerState.READY);
                } else if (getDesiredState() == KickerState.IDLE) {
                    transitionTo(KickerState.IDLE);
                }

                break;
            case READY:
                if (getDesiredState() == KickerState.STOWED) {
                    transitionTo(KickerState.STOWED);
                } else if (getDesiredState() == KickerState.READY_REVERSE) {
                    transitionTo(KickerState.READY_REVERSE);
                } else if (getDesiredState() == KickerState.IDLE) {
                    transitionTo(KickerState.IDLE);
                }

                break;
        }

        double kickerVoltage = 0.0;
        switch (getCurrentState()) {
            case IDLE:
                kickerVoltage = 0.0;
                break;
            case STOWED:
                kickerVoltage = 0.0;
                break;
            case READY_REVERSE:
                kickerVoltage = -Constants.KickerConstants.KICKER_MOTOR_VOLTAGE.in(Volt);
                break;
            case READY:
                kickerVoltage = Constants.KickerConstants.KICKER_MOTOR_VOLTAGE.in(Volt);
                break;
            default:
                kickerVoltage = 0.0;
                System.err.println("Kicker in unknown state: " + getCurrentState());
                break;
        }

        kickerVoltage = MathUtil.clamp(kickerVoltage, -12, 12);
        io.setMotorVoltage(kickerVoltage);

        checkCanHealth();

        SmartDashboard.putNumber("Kicker/Voltage", kickerVoltage);

        SmartDashboard.putNumber("Kicker/Current", io.getMotorCurrent());

        SmartDashboard.putString("Kicker/Current State", getCurrentState().name());
        SmartDashboard.putString("Kicker/Desired State", getDesiredState().name());
    }
}
