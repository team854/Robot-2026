package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.SubsystemStateMachine;
import frc.robot.subsystems.turret.ShooterSubsystem.ShooterState;

public class IntakeDeploymentSubsystem extends SubsystemStateMachine<frc.robot.subsystems.intake.IntakeDeploymentSubsystem.IntakeDeploymentState> {
    public enum IntakeDeploymentState {
        UNKNOWN,
        RETRACTED,
        DEPLOYED,
        RETRACTING,
        DEPLOYING
    }

    private final IntakeDeploymentIO io;

    public IntakeDeploymentSubsystem(IntakeDeploymentIO io) {
        super(IntakeDeploymentState.UNKNOWN);

        this.io = io;
    }

    @Override
    public void periodic() {
        // Safety Check as the desired state should only ever RETRACTED, OR DEPLOYED
        if (getDesiredState() == IntakeDeploymentState.UNKNOWN || getDesiredState() == IntakeDeploymentState.RETRACTING || getDesiredState() == IntakeDeploymentState.DEPLOYING) {
            setDesiredState(IntakeDeploymentState.RETRACTED);
        }

        switch (getCurrentState()) {
            case UNKNOWN:

                if (io.getDeployedSensor()) {
                    transitionTo(IntakeDeploymentState.DEPLOYED);
                } else if (io.getRetractedSensor()) {
                    transitionTo(IntakeDeploymentState.RETRACTED);
                } else if (getDesiredState() == IntakeDeploymentState.DEPLOYED) {
                    transitionTo(IntakeDeploymentState.DEPLOYING);
                }
                break;
            case RETRACTED:

                if (getDesiredState() == IntakeDeploymentState.DEPLOYED) {
                    transitionTo(IntakeDeploymentState.DEPLOYING);
                } else if (!io.getRetractedSensor()) {
                    transitionTo(IntakeDeploymentState.RETRACTING);
                }
                break;

            case DEPLOYED:

                if (getDesiredState() == IntakeDeploymentState.RETRACTED) {
                    transitionTo(IntakeDeploymentState.RETRACTING);
                } else if (!io.getDeployedSensor()) {
                    transitionTo(IntakeDeploymentState.DEPLOYING);
                } 
                break;

            case RETRACTING:

                if (getDesiredState() == IntakeDeploymentState.DEPLOYED) {
                    transitionTo(IntakeDeploymentState.DEPLOYING);
                } else if (io.getRetractedSensor()) {
                    transitionTo(IntakeDeploymentState.RETRACTED);
                }
                break;
            case DEPLOYING:

                if (getDesiredState() == IntakeDeploymentState.RETRACTED) {
                    transitionTo(IntakeDeploymentState.RETRACTING);
                } else if (io.getDeployedSensor()) {
                    transitionTo(IntakeDeploymentState.DEPLOYED);
                }
                break;
        }

        double intakeDeploymentVoltage = 0;

        switch (getCurrentState()) {
            case UNKNOWN:
                intakeDeploymentVoltage = Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_VOLTAGE.in(Volt);
                break;
            case RETRACTED:
                intakeDeploymentVoltage = 0.0;
                break;
            case DEPLOYED:
                intakeDeploymentVoltage = 0.0;
                break;
            case RETRACTING:
                intakeDeploymentVoltage = Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_VOLTAGE.in(Volt);
                break;
            case DEPLOYING:
                intakeDeploymentVoltage = -Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_VOLTAGE.in(Volt);
                break;
        }

        io.setDeploymentMotorVoltage(intakeDeploymentVoltage);
    }
}
