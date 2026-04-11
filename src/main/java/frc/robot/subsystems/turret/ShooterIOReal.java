package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
    private final SparkMaxConfig shooterConfig1;
    //private final SparkMaxConfig shooterConfig2;
    private final SparkMax shooterMotor1;
    //private final SparkMax shooterMotor2;

    private final RelativeEncoder shooterEncoder1;
    //private final RelativeEncoder shooterEncoder2;

    private final SparkClosedLoopController shooterClosedLoop1;

    public ShooterIOReal() {
        shooterMotor1 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
        //shooterMotor2 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);

        shooterEncoder1 = shooterMotor1.getEncoder();
        //shooterEncoder2 = shooterMotor2.getEncoder();

        shooterClosedLoop1 = shooterMotor1.getClosedLoopController();

        double velocityConversionFactor = (1.0 / 60.0) * Constants.ShooterConstants.SHOOTER_GEAR_RATIO; // Convert from RPM to RPS

        shooterConfig1 = new SparkMaxConfig();
        shooterConfig1.idleMode(IdleMode.kCoast);
        shooterConfig1.inverted(Constants.ShooterConstants.SHOOTER_MOTOR_1_INVERTED);
        shooterConfig1.encoder.velocityConversionFactor(velocityConversionFactor);
        shooterConfig1.closedLoop.pid(
            Constants.ShooterConstants.SHOOTER_P,
            Constants.ShooterConstants.SHOOTER_I,
            Constants.ShooterConstants.SHOOTER_D
        );
        shooterConfig1.closedLoop.feedForward.apply(
            new FeedForwardConfig()
                .kS(Constants.ShooterConstants.SHOOTER_S.in(Volt))
                .kV(Constants.ShooterConstants.SHOOTER_V)
                .kA(Constants.ShooterConstants.SHOOTER_A)
        );
        shooterMotor1.configure(shooterConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        
        shooterClosedLoop1.setSetpoint(targetRPS, ControlType.kVelocity);
    }

    @Override
    public void setMotorsVoltage(double voltage) {
        shooterMotor1.setVoltage(voltage);
        //shooterMotor2.setVoltage(voltage);
    }

    @Override
    public double getMotor1RPS() {
        return shooterEncoder1.getVelocity();
    }

    @Override
    public double getMotor2RPS() {
        //return shooterEncoder2.getVelocity();
        return shooterEncoder1.getVelocity();
    }

    @Override
    public boolean checkCANError() {
        shooterMotor1.getBusVoltage();
        if (shooterMotor1.getFaults().can == true) {
            return true;
        }

        return false;
    }
}
