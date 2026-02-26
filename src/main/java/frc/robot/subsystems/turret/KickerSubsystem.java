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

    private SparkMaxConfig kickerConfig;
    private SparkMax kickerMotor;

    public KickerSubsystem() {
        super(KickerState.IDLE);

        if (Constants.KickerConstants.ENABLED) {
            kickerMotor = new SparkMax(Constants.KickerConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

            kickerConfig = new SparkMaxConfig();
            kickerConfig.idleMode(IdleMode.kCoast);
            kickerConfig.inverted(Constants.KickerConstants.KICKER_MOTOR_INVERTED);
            kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    @Override
    public void periodic() {
        if (Constants.KickerConstants.ENABLED == false) {
            return;
        }

        switch (getCurrentState()) {
            case IDLE:
                kickerMotor.set(0);

                if (getDesiredState() == KickerState.READY_REVERSE) {
                    transitionTo(KickerState.READY_REVERSE);
                } else if (getDesiredState() == KickerState.READY) {
                    transitionTo(KickerState.READY);
                }
                break;
            case READY_REVERSE:
                kickerMotor.set(-1);
                
                if (getDesiredState() == KickerState.READY) {
                    transitionTo(KickerState.READY);
                } else if (getDesiredState() == KickerState.IDLE) {
                    transitionTo(KickerState.IDLE);
                }

                break;
            case READY:
                kickerMotor.set(1);
                
                if (getDesiredState() == KickerState.READY_REVERSE) {
                    transitionTo(KickerState.READY_REVERSE);
                } else if (getDesiredState() == KickerState.IDLE) {
                    transitionTo(KickerState.IDLE);
                }

                break;
        }
    }
}
