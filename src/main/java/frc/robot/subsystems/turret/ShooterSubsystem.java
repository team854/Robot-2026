package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMaxConfig shooterConfig1 = new SparkMaxConfig();
    private final SparkMaxConfig shooterConfig2 = new SparkMaxConfig();
    private final SparkMax shooterMotor1 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
    private final SparkMax shooterMotor2 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);

    private final RelativeEncoder shooterEncoder1 = shooterMotor1.getEncoder();
    private final RelativeEncoder shooterEncoder2 = shooterMotor2.getEncoder();

    private final ProfiledPIDController pid = new ProfiledPIDController(
        Constants.ShooterConstants.SHOOTER_P,
        Constants.ShooterConstants.SHOOTER_I, 
        Constants.ShooterConstants.SHOOTER_D,
        new TrapezoidProfile.Constraints(
            Constants.ShooterConstants.SHOOTER_MAX_VELOCITY.in(RotationsPerSecond),
            Constants.ShooterConstants.SHOOTER_MAX_ACCELERATION.in(RotationsPerSecondPerSecond)
        )
    );

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.ShooterConstants.SHOOTER_S.in(Volts),
        Constants.ShooterConstants.SHOOTER_V.in(Volts), // Unit is V/(rad/s)
        Constants.ShooterConstants.SHOOTER_A.in(Volts) // Unit is V/(rad/s^2)
    );

    private AngularVelocity targetSpeed = RotationsPerSecond.of(0);
    
    public ShooterSubsystem() {
        double velocityConversionFactor = (1.0 / 60.0) * Constants.ShooterConstants.SHOOTER_GEAR_RATIO;

        shooterConfig1.idleMode(IdleMode.kCoast);
        shooterConfig1.inverted(Constants.ShooterConstants.SHOOTER_MOTOR_1_INVERTED);
        shooterConfig1.encoder.velocityConversionFactor(velocityConversionFactor);
        shooterMotor1.configure(shooterConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        shooterConfig2.idleMode(IdleMode.kCoast);
        shooterConfig2.inverted(Constants.ShooterConstants.SHOOTER_MOTOR_2_INVERTED);
        shooterConfig2.encoder.velocityConversionFactor(velocityConversionFactor);
        shooterMotor2.configure(shooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSpeed(AngularVelocity speed) {
        targetSpeed = speed;

        
    }
}
