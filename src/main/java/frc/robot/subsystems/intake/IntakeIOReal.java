package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private final SparkMax intakeMotor1;
    private final SparkMax intakeMotor2;

    private final SparkMaxConfig intakeConfig1;
    private final SparkMaxConfig intakeConfig2;

    public IntakeIOReal() {
        intakeMotor1 = new SparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID_1, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID_2, MotorType.kBrushless);

        intakeConfig1 = new SparkMaxConfig();
        intakeConfig1.idleMode(IdleMode.kCoast);
        intakeConfig1.inverted(Constants.IntakeConstants.INTAKE_MOTOR_INVERTED_1);
        intakeConfig1.smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_MOTOR_CURRENT_LIMIT.in(Amp));
        intakeMotor1.configure(intakeConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeConfig2 = new SparkMaxConfig();
        intakeConfig2.idleMode(IdleMode.kCoast);
        intakeConfig2.inverted(Constants.IntakeConstants.INTAKE_MOTOR_INVERTED_2);
        intakeConfig2.smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_MOTOR_CURRENT_LIMIT.in(Amp));
        intakeMotor2.configure(intakeConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    @Override
    public void setIntakeMotorVoltage(double voltage) {
        intakeMotor1.setVoltage(voltage);
        intakeMotor2.setVoltage(voltage);
    }

    @Override
    public boolean checkCANError() {
        intakeMotor1.getBusVoltage();
        if (intakeMotor1.getFaults().can == true) {
            return true;
        }

        intakeMotor2.getBusVoltage();
        if (intakeMotor2.getFaults().can == true) {
            return true;
        }

        return false;
    }
}
