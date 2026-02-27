package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.SubsystemStateMachine;

public class SpindexerSubsystem extends SubsystemStateMachine<frc.robot.subsystems.spindexer.SpindexerSubsystem.SpindexerState> {

    public enum SpindexerState {
        IDLE,
        READY_REVERSE,
        READY,
    }

    private final SpindexerIO io;

    public SpindexerSubsystem(SpindexerIO io) {
        super(SpindexerState.IDLE);

        this.io = io;
    }

    @Override
    public void periodic() {
        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == SpindexerState.READY_REVERSE) {
                    transitionTo(SpindexerState.READY_REVERSE);
                } else if (getDesiredState() == SpindexerState.READY) {
                    transitionTo(SpindexerState.READY);
                }
                break;
            case READY_REVERSE:
                if (getDesiredState() == SpindexerState.READY) {
                    transitionTo(SpindexerState.READY);
                } else if (getDesiredState() == SpindexerState.IDLE) {
                    transitionTo(SpindexerState.IDLE);
                }

                break;
            case READY:
                if (getDesiredState() == SpindexerState.READY_REVERSE) {
                    transitionTo(SpindexerState.READY_REVERSE);
                } else if (getDesiredState() == SpindexerState.IDLE) {
                    transitionTo(SpindexerState.IDLE);
                }

                break;
        }

        switch (getCurrentState()) {
            case IDLE:
                io.setMotorVoltage(0);
                break;
            case READY_REVERSE:
                io.setMotorVoltage(-12);
                break;
            case READY:
                io.setMotorVoltage(12);
                break;
        }
    }
}
