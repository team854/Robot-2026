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

    private final SparkMax intakeDeploymentMotor;
    private final SparkMaxConfig intakeDeploymentConfig;

    private final DigitalInput intakeDeployedSensor;
    private final DigitalInput intakeRetractedSensor;

    public IntakeDeploymentIOReal() {
        intakeDeployedSensor = new DigitalInput(Constants.IntakeConstants.INTAKE_DEPLOYED_SENSOR_ID);
        intakeRetractedSensor = new DigitalInput(Constants.IntakeConstants.INTAKE_RETRACTED_SENSOR_ID);

        intakeDeploymentMotor = new SparkMax(Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_ID, MotorType.kBrushless);

        intakeDeploymentConfig = new SparkMaxConfig();
        intakeDeploymentConfig.idleMode(IdleMode.kBrake);
        intakeDeploymentConfig.inverted(Constants.IntakeConstants.INTAKE_DEPLOYMENT_MOTOR_INVERTED);
        intakeDeploymentConfig.smartCurrentLimit(20);
        intakeDeploymentMotor.configure(intakeDeploymentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setDeploymentMotorVoltage(double voltage) {
        intakeDeploymentMotor.setVoltage(voltage);
    }

    @Override
    public boolean getRetractedSensor() {
        return !intakeRetractedSensor.get();
    }

    @Override
    public boolean getDeployedSensor() {
        return !intakeDeployedSensor.get();
    }
}
