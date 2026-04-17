package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Constants.KickerConstants;

public class KickerIOReal implements KickerIO {
    
    private final SparkMaxConfig kickerConfig;
    private final SparkMax kickerMotor;

    public KickerIOReal() {
        kickerMotor = new SparkMax(Constants.KickerConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

        kickerConfig = new SparkMaxConfig();
        kickerConfig.idleMode(IdleMode.kBrake);
        kickerConfig.inverted(Constants.KickerConstants.KICKER_MOTOR_INVERTED);
        kickerConfig.smartCurrentLimit((int) KickerConstants.KICKER_MOTOR_CURRENT_LIMIT.in(Amp)); 
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        kickerMotor.setVoltage(voltage);
    }

    @Override
    public double getMotorCurrent() {
        return kickerMotor.getOutputCurrent();
    }

    @Override
    public boolean checkCANError() {
        kickerMotor.getBusVoltage();
        if (kickerMotor.getFaults().can == true) {
            return true;
        }

        return false;
    }
}
