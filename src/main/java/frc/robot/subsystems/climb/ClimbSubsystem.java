package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volt;

import frc.robot.Constants;
import frc.robot.libraries.SubsystemStateMachine;

public class ClimbSubsystem extends SubsystemStateMachine<frc.robot.subsystems.climb.ClimbSubsystem.ClimbState> {
    public enum ClimbState {
        IDLE,
        CLIMBING,
        CLIMBED
    }

    private final ClimbIO io;

    public ClimbSubsystem(ClimbIO io) {
        super(ClimbState.IDLE, null);

        this.io = io;
    }

    @Override
    public void periodic() {
        updateDesiredState();

        // Safety Check as the desired state should only ever be IDLE or CLIMBED
        if (getDesiredState() == ClimbState.CLIMBING) {
            requestDesiredState(ClimbState.IDLE, 5);
        }

        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == ClimbState.CLIMBED) {
                    transitionTo(ClimbState.CLIMBING);
                }
                break;
            case CLIMBING:
                if (getDesiredState() == ClimbState.IDLE) {
                    transitionTo(ClimbState.IDLE);
                } else if (io.getClimbedSensor()) {
                    transitionTo(ClimbState.CLIMBED);
                }
                break;
            case CLIMBED:
                if (!io.getClimbedSensor()) {
                    transitionTo(ClimbState.CLIMBING);
                }
                break;
        }

        double climbVoltage = 0;
        switch (getCurrentState()) {
            case IDLE:
                climbVoltage = 0;
                break;
            case CLIMBING:
                climbVoltage = Constants.ClimbConstants.CLIMB_MOTOR_VOLTAGE.in(Volt);
                break;
            case CLIMBED:
                climbVoltage = 0;
                break;
        }

        io.setClimbMotorVoltage(climbVoltage);
    }
}
