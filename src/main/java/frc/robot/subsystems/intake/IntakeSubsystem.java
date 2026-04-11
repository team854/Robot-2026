package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ErrorConstants;
import frc.robot.RobotContainer;
import frc.robot.libraries.SubsystemStateMachine;

public class IntakeSubsystem extends SubsystemStateMachine<frc.robot.subsystems.intake.IntakeSubsystem.IntakeState>{
    public enum IntakeState {
        IDLE,
        READY_REVERSE,
        READY
    }

    private final IntakeIO io;

    private double lastErrorTimestamp = Double.NEGATIVE_INFINITY;

    public IntakeSubsystem(IntakeIO io) {
        super(IntakeState.IDLE, IntakeState.IDLE);

        if (io == null) {
            throw new IllegalArgumentException("IntakeIO cannot be null");
        }

        this.io = io;
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
    public void statePeriodic() {

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
            default:
                intakeVoltage = 0.0;
                System.err.println("Intake in unknown state: " + getCurrentState());
                break;
        }

        intakeVoltage = MathUtil.clamp(intakeVoltage, -12, 12);
        io.setIntakeMotorVoltage(intakeVoltage);

        checkCanHealth();

        SmartDashboard.putNumber("Intake/Voltage", intakeVoltage);

        SmartDashboard.putString("Intake/Current State", getCurrentState().name());
        SmartDashboard.putString("Intake/Desired State", getDesiredState().name());
    }
}
