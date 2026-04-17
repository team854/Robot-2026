package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class TurretIOKrakenReal implements TurretIO {
    
    private final SparkMax turretYawMotor; 
    private final RelativeEncoder turretYawEncoder;
    private final AbsoluteEncoder turretYawAbsoluteEncoder;
    private final SparkMaxConfig turretYawConfig;
    private final double yawConversionFactor = (2.0 * Math.PI) / Constants.TurretConstants.TURRET_YAW_GEAR_RATIO;
    private final double relativeStepSize = (2 * Math.PI) / (Constants.TurretConstants.TURRET_YAW_COUNTS_PER_REV * Constants.TurretConstants.TURRET_YAW_GEAR_RATIO);
    private double yawAbsoluteOffset = 0;
    
    private final TalonFX turretPitchMotor;
    private final double pitchConversionFactor = (2.0 * Math.PI) / Constants.TurretConstants.TURRET_PITCH_GEAR_RATIO;
    private final TalonFXConfiguration turretPitchConfig;

    private final DigitalInput yawHomingSensor;
    private final Counter yawHomingCounter;

    private final VoltageOut pitchVoltageRequest = new VoltageOut(0);
    
    public TurretIOKrakenReal() {
        turretYawMotor = new SparkMax(Constants.TurretConstants.TURRET_YAW_MOTOR_ID, MotorType.kBrushless);
        turretPitchMotor = new TalonFX(Constants.TurretConstants.TURRET_PITCH_MOTOR_ID);

        turretYawEncoder = turretYawMotor.getEncoder();
        turretYawAbsoluteEncoder = turretYawMotor.getAbsoluteEncoder();

        // Configure motors
        
        
        turretYawConfig = new SparkMaxConfig();
        turretYawConfig.inverted(Constants.TurretConstants.TURRET_YAW_MOTOR_INVERTED);
        turretYawConfig.idleMode(IdleMode.kBrake);
        turretYawConfig.encoder.positionConversionFactor(yawConversionFactor);
        turretYawConfig.encoder.velocityConversionFactor(yawConversionFactor / 60.0);
        turretYawConfig.signals.primaryEncoderPositionPeriodMs(10);
        turretYawConfig.absoluteEncoder.positionConversionFactor(yawConversionFactor);
        turretYawConfig.absoluteEncoder.inverted(Constants.TurretConstants.TURRET_YAW_ABSOLUTE_ENCODER_INVERTED);
        turretYawConfig.signals.absoluteEncoderPositionPeriodMs(10);
        turretYawConfig.smartCurrentLimit((int) Constants.TurretConstants.TURRET_YAW_MOTOR_CURRENT_LIMIT.in(Amp));
        turretYawMotor.configure(turretYawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
        turretPitchConfig = new TalonFXConfiguration();
        turretPitchConfig.MotorOutput.Inverted = Constants.TurretConstants.TURRET_PITCH_MOTOR_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        turretPitchConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turretPitchMotor.getConfigurator().apply(turretPitchConfig);
        

        yawHomingSensor = new DigitalInput(Constants.TurretConstants.TURRET_YAW_HOMING_SENSOR_DIO);
        yawHomingCounter = new Counter(yawHomingSensor);
        yawHomingCounter.setUpSourceEdge(true,false);

        setYawEncoderPosition(turretYawEncoder.getPosition());
        turretPitchMotor.setPosition(0);
    }

    @Override
    public void setYawMotorVoltage(double voltage) {
        turretYawMotor.setVoltage(voltage);
    }

    
    @Override
    public void setPitchMotorVoltage(double voltage) {
        turretPitchMotor.setControl(pitchVoltageRequest.withOutput(voltage));
    }
    

    @Override
    public double getYawRadians() {
        
        double relativeEncoder = turretYawEncoder.getPosition();
        double absoluteEncoder = MathUtil.inputModulus(turretYawAbsoluteEncoder.getPosition() - yawAbsoluteOffset, 0, yawConversionFactor);

        double difference = MathUtil.inputModulus(absoluteEncoder - relativeEncoder, -yawConversionFactor / 2.0, yawConversionFactor / 2.0);
        
        // Two relative encoder tick correction window because at high speeds the relative and absolute can become too desynced
        difference = MathUtil.clamp(difference, -relativeStepSize * Constants.TurretConstants.TURRET_YAW_FUSION_MULTIPLIER, relativeStepSize * Constants.TurretConstants.TURRET_YAW_FUSION_MULTIPLIER);


        return relativeEncoder + difference;
    }

    
    @Override
    public double getPitchRadians() {
        return (Constants.TurretConstants.TURRET_PITCH_LOWER_LIMIT.in(Radian) + (turretPitchMotor.getPosition().getValueAsDouble() * pitchConversionFactor));
    }

    @Override
    public double getRawYawRadians() {
        return turretYawEncoder.getPosition();
    }

    @Override
    public double getRawAbsoluteYawRadians() {
        return turretYawAbsoluteEncoder.getPosition();
    }

    @Override
    public void setYawEncoderPosition(double position) {
        
        turretYawEncoder.setPosition(position);

        double expectedPosition = MathUtil.inputModulus(position, 0, yawConversionFactor);

        yawAbsoluteOffset = MathUtil.inputModulus(turretYawAbsoluteEncoder.getPosition() - expectedPosition, -yawConversionFactor / 2.0, yawConversionFactor / 2.0);
        
    }

    @Override
    public boolean getHomingSensor() {
        return !yawHomingSensor.get();
        //return false;
    }

    @Override
    public boolean getHomingCounter() {
        return yawHomingCounter.get() > 0;
        //return false;
    }

    @Override
    public void resetHomingCounter() {
        yawHomingCounter.reset();
    }

    @Override
    public void resetPitchPosition() {
        turretPitchMotor.setPosition(0);
    }

    @Override
    public boolean checkCANError() {
        if (!turretPitchMotor.isConnected()) {
            return true;
        }

        turretYawMotor.getBusVoltage();
        if (turretYawMotor.getFaults().can == true) {
            return true;
        }

        return false;
    }
    
}