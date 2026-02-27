package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private final SparkMax intakeMotor;
    private final SparkMaxConfig intakeConfig;

    public IntakeIOReal() {
        intakeMotor = new SparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeConfig.inverted(Constants.IntakeConstants.INTAKE_MOTOR_INVERTED);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    @Override
    public void setIntakeMotorVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }
}
