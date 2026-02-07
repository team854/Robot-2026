package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMaxConfig shooterConfig1 = new SparkMaxConfig();
    private final SparkMaxConfig shooterConfig2 = new SparkMaxConfig();
    private final SparkMax shooterMotor1 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
    private final SparkMax shooterMotor2 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);

    private final RelativeEncoder shooterEncoder1 = shooterMotor1.getEncoder();
    private final RelativeEncoder shooterEncoder2 = shooterMotor2.getEncoder();

    private final ProfiledPIDController shooterPID = new ProfiledPIDController(
        Constants.ShooterConstants.SHOOTER_P,
        Constants.ShooterConstants.SHOOTER_I, 
        Constants.ShooterConstants.SHOOTER_D,
        new TrapezoidProfile.Constraints(
            Constants.ShooterConstants.SHOOTER_MAX_VELOCITY.in(RotationsPerSecond),
            Constants.ShooterConstants.SHOOTER_MAX_ACCELERATION.in(RotationsPerSecondPerSecond)
        )
    );

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
        Constants.ShooterConstants.SHOOTER_S.in(Volts),
        Constants.ShooterConstants.SHOOTER_V.in(Volts), // Unit is V/(rad/s)
        Constants.ShooterConstants.SHOOTER_A.in(Volts) // Unit is V/(rad/s^2)
    );

    private double shooterPrevSetpointVelocity = 0;

    private AngularVelocity targetSpeed = RotationsPerSecond.of(0);
    
    public ShooterSubsystem() {
        double velocityConversionFactor = 60 * Constants.ShooterConstants.SHOOTER_GEAR_RATIO;

        shooterConfig1.idleMode(IdleMode.kCoast);
        shooterConfig1.inverted(Constants.ShooterConstants.SHOOTER_MOTOR_1_INVERTED);
        shooterConfig1.encoder.velocityConversionFactor(velocityConversionFactor);
        shooterMotor1.configure(shooterConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        shooterConfig2.idleMode(IdleMode.kCoast);
        shooterConfig2.inverted(Constants.ShooterConstants.SHOOTER_MOTOR_2_INVERTED);
        shooterConfig2.encoder.velocityConversionFactor(velocityConversionFactor); // One of these might need to be inverted
        shooterMotor2.configure(shooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setTargetSpeed(AngularVelocity speed) {
        shooterPID.setGoal(speed.in(RotationsPerSecond));

        SmartDashboard.putNumber("Shooter/Target Speed", targetSpeed.in(RotationsPerSecond) / 60.0);
    }

    public AngularVelocity getTargetSpeed() {
        return RotationsPerSecond.of(shooterPID.getGoal().position);
    }

    public AngularVelocity getSpeed() {
        double speed1 = shooterEncoder1.getVelocity();
        double speed2 = shooterEncoder2.getVelocity();

        return RotationsPerSecond.of((speed1 + speed2) / 2.0);
    }

    @Override
    public void periodic() {
        double shooterVoltage = shooterPID.calculate(getSpeed().in(RotationsPerSecond));
        TrapezoidProfile.State shooterState = shooterPID.getSetpoint();
        shooterVoltage += shooterFF.calculateWithVelocities(shooterPrevSetpointVelocity, shooterState.velocity);
        shooterVoltage = MathUtil.clamp(
            shooterVoltage, 
           -10.0, 
           10.0
        );
        shooterMotor1.setVoltage(shooterVoltage);
        shooterMotor2.setVoltage(shooterVoltage);
        shooterPrevSetpointVelocity = shooterState.velocity;
    }
}
