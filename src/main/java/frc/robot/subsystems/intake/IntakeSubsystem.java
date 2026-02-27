package frc.robot.subsystems.intake;

import frc.robot.libraries.SubsystemStateMachine;

public class IntakeSubsystem extends SubsystemStateMachine<frc.robot.subsystems.intake.IntakeSubsystem.IntakeState>{
    public enum IntakeState {
        IDLE,
        READY_REVERSE,
        READY
    }

    private final IntakeIO io;

    public IntakeSubsystem(IntakeIO io) {
        super(IntakeState.IDLE);

        this.io = io;
    }

    @Override
    public void periodic() {
        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == IntakeState.READY_REVERSE) {
                    transitionTo(IntakeState.READY_REVERSE);
                } else if (getDesiredState() == IntakeState.READY) {
                    transitionTo(IntakeState.READY);
                }
                break;
            case READY_REVERSE:
                if (getDesiredState() == IntakeState.READY) {
                    transitionTo(IntakeState.READY);
                } else if (getDesiredState() == IntakeState.IDLE) {
                    transitionTo(IntakeState.IDLE);
                }

                break;
            case READY:
                if (getDesiredState() == IntakeState.READY_REVERSE) {
                    transitionTo(IntakeState.READY_REVERSE);
                } else if (getDesiredState() == IntakeState.IDLE) {
                    transitionTo(IntakeState.IDLE);
                }

                break;
        }

        switch (getCurrentState()) {
            case IDLE:
                io.setIntakeMotorVoltage(0);
                break;
            case READY_REVERSE:
                io.setIntakeMotorVoltage(-12);
                break;
            case READY:
                io.setIntakeMotorVoltage(12);
                break;
        }
    }
}
