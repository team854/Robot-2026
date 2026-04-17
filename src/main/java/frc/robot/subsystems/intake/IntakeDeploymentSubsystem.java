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

public class IntakeDeploymentSubsystem extends SubsystemStateMachine<frc.robot.subsystems.intake.IntakeDeploymentSubsystem.IntakeDeploymentState> {
    public enum IntakeDeploymentState {
        UNKNOWN,
        RETRACTED,
        DEPLOYED,
        RETRACTING,
        DEPLOYING
    }

    private final IntakeDeploymentIO io;

    private double lastErrorTimestamp = Double.NEGATIVE_INFINITY;

    public IntakeDeploymentSubsystem(IntakeDeploymentIO io) {
        super(IntakeDeploymentState.UNKNOWN, null);

        if (io == null) {
            throw new IllegalArgumentException("IntakeDeploymentIO cannot be null");
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
    public void statePeriodicBefore() {
        // Safety Check as the desired state should only ever be RETRACTED, OR DEPLOYED
        if (getDesiredState() == IntakeDeploymentState.UNKNOWN || getDesiredState() == IntakeDeploymentState.RETRACTING || getDesiredState() == IntakeDeploymentState.DEPLOYING) {
            requestDesiredState(IntakeDeploymentState.RETRACTED, 25);
        }
    }

    @Override
    public void statePeriodic() {
        

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
                intakeDeploymentVoltage = -Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_VOLTAGE.in(Volt);
                break;
            case RETRACTED:
                intakeDeploymentVoltage = -Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_HOLDING_VOLTAGE.in(Volt);
                break;
            case DEPLOYED:
                intakeDeploymentVoltage = Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_HOLDING_VOLTAGE.in(Volt);
                break;
            case RETRACTING:
                intakeDeploymentVoltage = -Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_VOLTAGE.in(Volt);
                break;
            case DEPLOYING:
                if (getStateTimer() > 2.0) {
                    intakeDeploymentVoltage = Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_FORCE_VOLTAGE.in(Volt);
                } else {
                    intakeDeploymentVoltage = Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_VOLTAGE.in(Volt);
                }
                
                break;
            default:
                intakeDeploymentVoltage = 0.0;
                System.err.println("IntakeDeployment in unknown state: " + getCurrentState());
                break;
        }

        intakeDeploymentVoltage = MathUtil.clamp(intakeDeploymentVoltage, -5, 5);
        io.setDeploymentMotorVoltage(intakeDeploymentVoltage);

        checkCanHealth();

        SmartDashboard.putNumber("Intake Deployment/Voltage", intakeDeploymentVoltage);

        SmartDashboard.putBoolean("Intake Deployment/Retracted Sensor", io.getRetractedSensor());
        SmartDashboard.putBoolean("Intake Deployment/Deployed Sensor", io.getDeployedSensor());

        SmartDashboard.putString("Intake Deployment/Current State", getCurrentState().name());
        SmartDashboard.putString("Intake Deployment/Desired State", getDesiredState().name());
    }
}
