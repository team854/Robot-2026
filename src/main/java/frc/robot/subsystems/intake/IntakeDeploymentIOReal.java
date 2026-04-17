package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeDeploymentIOReal implements IntakeDeploymentIO {

    private final SparkMax intakeDeploymentMotor1;
    private final SparkMaxConfig intakeDeploymentConfig1;

    private final SparkMax intakeDeploymentMotor2;
    private final SparkMaxConfig intakeDeploymentConfig2;

    private final DigitalInput intakeDeployedSensor;
    private final DigitalInput intakeRetractedSensor;

    public IntakeDeploymentIOReal() {
        intakeDeployedSensor = new DigitalInput(Constants.IntakeConstants.INTAKE_DEPLOYED_SENSOR_DIO);
        intakeRetractedSensor = new DigitalInput(Constants.IntakeConstants.INTAKE_RETRACTED_SENSOR_DIO);

        intakeDeploymentMotor1 = new SparkMax(Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_ID_1, MotorType.kBrushless);
        intakeDeploymentMotor2 = new SparkMax(Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_ID_2, MotorType.kBrushless);

        intakeDeploymentConfig1 = new SparkMaxConfig();
        intakeDeploymentConfig1.idleMode(IdleMode.kBrake);
        intakeDeploymentConfig1.inverted(Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_INVERTED_1);
        intakeDeploymentConfig1.smartCurrentLimit(20);
        intakeDeploymentMotor1.configure(intakeDeploymentConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeDeploymentConfig2 = new SparkMaxConfig();
        intakeDeploymentConfig2.idleMode(IdleMode.kBrake);
        intakeDeploymentConfig2.inverted(Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_INVERTED_2);
        intakeDeploymentConfig2.smartCurrentLimit(20);
        intakeDeploymentMotor2.configure(intakeDeploymentConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setDeploymentMotorVoltage(double voltage) {
        intakeDeploymentMotor1.setVoltage(voltage);
        intakeDeploymentMotor2.setVoltage(voltage);
    }

    @Override
    public boolean getRetractedSensor() {
        return !intakeRetractedSensor.get();
    }

    @Override
    public boolean getDeployedSensor() {
        return !intakeDeployedSensor.get();
    }

    @Override
    public boolean checkCANError() {
        intakeDeploymentMotor1.getBusVoltage();
        if (intakeDeploymentMotor1.getFaults().can == true) {
            return true;
        }

        intakeDeploymentMotor2.getBusVoltage();
        if (intakeDeploymentMotor2.getFaults().can == true) {
            return true;
        }

        return false;
    }
}
