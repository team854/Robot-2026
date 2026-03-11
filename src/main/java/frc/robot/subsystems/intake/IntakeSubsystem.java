package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.libraries.SubsystemStateMachine;

public class IntakeSubsystem extends SubsystemStateMachine<frc.robot.subsystems.intake.IntakeSubsystem.IntakeState>{
    public enum IntakeState {
        IDLE,
        READY_REVERSE,
        READY
    }

    private final IntakeIO io;

    public IntakeSubsystem(IntakeIO io) {
        super(IntakeState.IDLE, IntakeState.IDLE);

        this.io = io;
    }

    @Override
    public void periodic() {
        updateDesiredState();

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

        double intakeVoltage = 0.0;
        switch (getCurrentState()) {
            case IDLE:
                intakeVoltage = 0.0;
                break;
            case READY_REVERSE:
                intakeVoltage = -Constants.IntakeConstants.INTAKE_MOTOR_VOLTAGE.in(Volt);
                break;
            case READY:
                intakeVoltage = Constants.IntakeConstants.INTAKE_MOTOR_VOLTAGE.in(Volt);
                break;
        }

        io.setIntakeMotorVoltage(intakeVoltage);

        SmartDashboard.putNumber("Intake/Voltage", intakeVoltage);

        SmartDashboard.putString("Intake/Current State", getCurrentState().name());
        SmartDashboard.putString("Intake/Desired State", getDesiredState().name());
    }
}
