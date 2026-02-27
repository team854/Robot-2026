package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class KickerIOReal implements KickerIO {
    
    private final SparkMaxConfig kickerConfig;
    private final SparkMax kickerMotor;

    public KickerIOReal() {
        kickerMotor = new SparkMax(Constants.KickerConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

        kickerConfig = new SparkMaxConfig();
        kickerConfig.idleMode(IdleMode.kCoast);
        kickerConfig.inverted(Constants.KickerConstants.KICKER_MOTOR_INVERTED);
        kickerConfig.smartCurrentLimit(40); 
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        kickerMotor.setVoltage(voltage);
    }
}
