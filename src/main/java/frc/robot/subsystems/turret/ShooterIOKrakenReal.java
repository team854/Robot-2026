package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ShooterIOKrakenReal implements ShooterIO {
    private final TalonFX shooterMotor1;
    private final TalonFXConfiguration shooterConfig1;

    //private final SparkMax shooterMotor2;
    //private final SparkMaxConfig shooterConfig2;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public ShooterIOKrakenReal() {
        shooterMotor1 = new TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_1_ID);
        //shooterMotor2 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);

        shooterConfig1 = new TalonFXConfiguration();
        shooterConfig1.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig1.MotorOutput.Inverted = Constants.ShooterConstants.SHOOTER_MOTOR_1_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        shooterConfig1.Feedback.SensorToMechanismRatio = 1.0 / Constants.ShooterConstants.SHOOTER_GEAR_RATIO;
        shooterConfig1.Slot0.kP = Constants.ShooterConstants.SHOOTER_P;
        shooterConfig1.Slot0.kI = Constants.ShooterConstants.SHOOTER_I;
        shooterConfig1.Slot0.kD = Constants.ShooterConstants.SHOOTER_D;
        shooterConfig1.Slot0.kS = Constants.ShooterConstants.SHOOTER_S.in(Volt);
        shooterConfig1.Slot0.kV = Constants.ShooterConstants.SHOOTER_V;
        shooterConfig1.Slot0.kA = Constants.ShooterConstants.SHOOTER_A;
        shooterMotor1.getConfigurator().apply(shooterConfig1);

        /*
        shooterConfig2 = new SparkMaxConfig();
        shooterConfig2.idleMode(IdleMode.kCoast);
        shooterConfig2.inverted(Constants.ShooterConstants.SHOOTER_MOTOR_2_INVERTED);
        shooterConfig2.encoder.velocityConversionFactor(velocityConversionFactor); // One of these might need to be inverted
        shooterMotor2.configure(shooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        */
    }

    @Override
    public void setClosedVelocity(double targetRPS) {
        shooterMotor1.setControl(velocityRequest.withVelocity(targetRPS));
    }

    @Override
    public void setMotorsVoltage(double voltage) {
        shooterMotor1.setControl(voltageRequest.withOutput(voltage));
        //shooterMotor2.setVoltage(voltage);
    }

    @Override
    public double getMotor1RPS() {
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    @Override
    public double getMotor2RPS() {
        //return shooterEncoder2.getVelocity();
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    @Override
    public boolean checkCANError() {
        if (!shooterMotor1.isConnected()) {
            return true;
        }

        return false;
    }
}
