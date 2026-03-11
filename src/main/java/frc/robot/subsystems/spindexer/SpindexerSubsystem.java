package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.SubsystemStateMachine;

public class SpindexerSubsystem extends SubsystemStateMachine<frc.robot.subsystems.spindexer.SpindexerSubsystem.SpindexerState> {

    public enum SpindexerState {
        IDLE,
        READY_REVERSE,
        READY,
    }

    private final SpindexerIO io;

    public SpindexerSubsystem(SpindexerIO io) {
        super(SpindexerState.IDLE, SpindexerState.IDLE);

        this.io = io;
    }

    @Override
    public void periodic() {
        updateDesiredState();

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

        double spindexerVoltage = 0.0;
        switch (getCurrentState()) {
            case IDLE:
                spindexerVoltage = 0;
                break;
            case READY_REVERSE:
                spindexerVoltage = -Constants.SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE.in(Volt);
                break;
            case READY:
                spindexerVoltage = Constants.SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE.in(Volt);
                
                break;
        }

        io.setMotorVoltage(spindexerVoltage);

        SmartDashboard.putNumber("Spindexer/Voltage", spindexerVoltage);

        SmartDashboard.putString("Spindexer/Current State", getCurrentState().name());
        SmartDashboard.putString("Spindexer/Desired State", getDesiredState().name());
    }
}
