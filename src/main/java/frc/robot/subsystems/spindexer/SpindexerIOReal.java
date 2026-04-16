package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class SpindexerIOReal implements SpindexerIO {
    private final SparkMaxConfig spindexerConfig;
    private final SparkMax spindexerMotor;

    public SpindexerIOReal() {
        spindexerMotor = new SparkMax(Constants.SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

        spindexerConfig = new SparkMaxConfig();
        spindexerConfig.idleMode(IdleMode.kBrake);
        spindexerConfig.inverted(Constants.SpindexerConstants.SPINDEXER_MOTOR_INVERTED);
        spindexerConfig.smartCurrentLimit(40); 
        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        spindexerMotor.setVoltage(voltage);
    }

    @Override
    public double getMotorCurrent() {
        return spindexerMotor.getOutputCurrent();
    }

    @Override
    public boolean checkCANError() {
        spindexerMotor.getBusVoltage();
        if (spindexerMotor.getFaults().can == true) {
            return true;
        }

        return false;
    }
}
