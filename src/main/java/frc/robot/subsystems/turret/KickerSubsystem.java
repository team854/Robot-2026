package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volt;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        super(KickerState.IDLE, KickerState.IDLE);

        this.io = io;
    }

    @Override
    public void periodic() {
        updateDesiredState();
        
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

        double kickerVoltage = 0.0;
        switch (getCurrentState()) {
            case IDLE:
                kickerVoltage = 0.0;
                break;
            case READY_REVERSE:
                kickerVoltage = -Constants.KickerConstants.KICKER_MOTOR_VOLTAGE.in(Volt);
                break;
            case READY:
                kickerVoltage = Constants.KickerConstants.KICKER_MOTOR_VOLTAGE.in(Volt);
                break;
        }

        io.setMotorVoltage(kickerVoltage);

        SmartDashboard.putNumber("Kicker/Voltage", kickerVoltage);

        SmartDashboard.putString("Kicker/Current State", getCurrentState().name());
        SmartDashboard.putString("Kicker/Desired State", getDesiredState().name());
    }
}
