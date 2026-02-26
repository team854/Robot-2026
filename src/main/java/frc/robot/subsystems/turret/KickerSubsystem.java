package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.SubsystemStateMachine;

public class KickerSubsystem extends SubsystemStateMachine<frc.robot.subsystems.turret.KickerSubsystem.KickerState> {

    public enum KickerState {
        IDLE,
        READY_REVERSE,
        READY,
    }

    private final KickerIO io;

    public KickerSubsystem(KickerIO io) {
        super(KickerState.IDLE);

        this.io = io;
    }

    @Override
    public void periodic() {
        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == KickerState.READY_REVERSE) {
                    transitionTo(KickerState.READY_REVERSE);
                } else if (getDesiredState() == KickerState.READY) {
                    transitionTo(KickerState.READY);
                }
                break;
            case READY_REVERSE:
                if (getDesiredState() == KickerState.READY) {
                    transitionTo(KickerState.READY);
                } else if (getDesiredState() == KickerState.IDLE) {
                    transitionTo(KickerState.IDLE);
                }

                break;
            case READY:
                if (getDesiredState() == KickerState.READY_REVERSE) {
                    transitionTo(KickerState.READY_REVERSE);
                } else if (getDesiredState() == KickerState.IDLE) {
                    transitionTo(KickerState.IDLE);
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
