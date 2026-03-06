package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radian;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class TurretIOReal implements TurretIO {

    private final SparkMax turretYawMotor; 
    private final RelativeEncoder turretYawEncoder;
    private final SparkMaxConfig turretYawConfig;
    private double turretYawZeroOffset = 0;

    /*
    private SparkMax turretPitchMotor;
    private RelativeEncoder turretPitchEncoder;
    private SparkMaxConfig turretPitchConfig;
    */

    private final DigitalInput yawHomingSensor;

    public TurretIOReal() {
        turretYawMotor = new SparkMax(Constants.TurretConstants.TURRET_YAW_MOTOR_ID, MotorType.kBrushless);
        //turretPitchMotor = new SparkMax(Constants.TurretConstants.TURRET_PITCH_MOTOR_ID, MotorType.kBrushless);

        turretYawEncoder = turretYawMotor.getEncoder();
        //turretPitchEncoder = turretPitchMotor.getEncoder();

        // Configure motors
        turretYawConfig = new SparkMaxConfig();
        turretYawConfig.inverted(Constants.TurretConstants.TURRET_YAW_MOTOR_INVERTED);
        double yawConversionFactor = (2.0 * Math.PI) / Constants.TurretConstants.TURRET_YAW_GEAR_RATIO;
        turretYawConfig.encoder.positionConversionFactor(yawConversionFactor);
        turretYawConfig.encoder.velocityConversionFactor(yawConversionFactor / 60.0);
        turretYawMotor.configure(turretYawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
        turretPitchConfig = new SparkMaxConfig();
        turretPitchConfig.inverted(Constants.TurretConstants.TURRET_PITCH_MOTOR_INVERTED);
        double pitchConversionFactor = (2.0 * Math.PI) / Constants.TurretConstants.TURRET_PITCH_GEAR_RATIO;
        turretPitchConfig.encoder.positionConversionFactor(pitchConversionFactor);
        turretPitchConfig.encoder.velocityConversionFactor(pitchConversionFactor / 60.0);
        turretPitchConfig.encoder.inverted(Constants.TurretConstants.TURRET_PITCH_ENCODER_INVERTED);
        turretPitchMotor.configure(turretPitchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        */

        yawHomingSensor = new DigitalInput(Constants.TurretConstants.TURRET_YAW_HOMING_SENSOR_DIO);


    }

    @Override
    public void setYawMotorVoltage(double voltage) {
        turretYawMotor.setVoltage(voltage);
    }

    
    @Override
    public void setPitchMotorVoltage(double voltage) {
        //turretPitchMotor.setVoltage(voltage);
    }
    

    @Override
    public double getYawRadians() {
        return turretYawEncoder.getPosition() - turretYawZeroOffset;
    }

    
    @Override
    public double getPitchRadians() {
        return 0.5;
        //return (Constants.TurretConstants.TURRET_PITCH_UPPER_LIMIT.in(Radian) - turretPitchEncoder.getPosition());
    }

    @Override
    public void setYawEncoderPosition(double position) {
        turretYawEncoder.setPosition(position);
    }

    @Override
    public boolean getHomingSensor() {
        return !yawHomingSensor.get();
    }
}