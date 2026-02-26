package frc.robot.subsystems.turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class TurretIOReal implements TurretIO {

    private SparkMax turretYawMotor; 
    private RelativeEncoder turretYawEncoder;
    private SparkMaxConfig turretYawConfig;

    private SparkMax turretPitchMotor;
    private AbsoluteEncoder turretPitchAbsoluteEncoder;
    private SparkMaxConfig turretPitchConfig;

    public TurretIOReal() {
        turretYawMotor = new SparkMax(Constants.TurretConstants.TURRET_YAW_MOTOR_ID, MotorType.kBrushless);
        turretPitchMotor = new SparkMax(Constants.TurretConstants.TURRET_PITCH_MOTOR_ID, MotorType.kBrushless);

        turretYawEncoder = turretYawMotor.getEncoder();
        turretPitchAbsoluteEncoder = turretPitchMotor.getAbsoluteEncoder();

        // Configure motors
        turretYawConfig = new SparkMaxConfig();
        turretYawConfig.inverted(Constants.TurretConstants.TURRET_YAW_MOTOR_INVERTED);
        double yawConversionFactor = (2.0 * Math.PI) / Constants.TurretConstants.TURRET_YAW_GEAR_RATIO;
        turretYawConfig.encoder.positionConversionFactor(yawConversionFactor);
        turretYawConfig.encoder.velocityConversionFactor(yawConversionFactor / 60.0);
        turretYawMotor.configure(turretYawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turretPitchConfig = new SparkMaxConfig();
        turretPitchConfig.inverted(Constants.TurretConstants.TURRET_PITCH_MOTOR_INVERTED);
        double pitchConversionFactor = (2.0 * Math.PI);
        turretPitchConfig.absoluteEncoder.positionConversionFactor(pitchConversionFactor);
        turretPitchConfig.absoluteEncoder.velocityConversionFactor(pitchConversionFactor / 60.0);
        turretPitchConfig.absoluteEncoder.inverted(Constants.TurretConstants.TURRET_PITCH_ENCODER_INVERTED);
        turretPitchConfig.absoluteEncoder.zeroOffset(Constants.TurretConstants.TURRET_PITCH_ZERO_OFFSET);
        turretPitchMotor.configure(turretPitchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setYawMotorVoltage(double voltage) {
        turretYawMotor.setVoltage(voltage);
    }

    @Override
    public void setPitchMotorVoltage(double voltage) {
        turretPitchMotor.setVoltage(voltage);
    }

    @Override
    public double getYawRadians() {
        return turretYawEncoder.getPosition();
    }

    @Override
    public double getPitchRadians() {
        return turretPitchAbsoluteEncoder.getPosition();
    }
}