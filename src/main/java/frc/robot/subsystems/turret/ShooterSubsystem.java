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
    private SparkMaxConfig shooterConfig1;
    private SparkMaxConfig shooterConfig2;
    private SparkMax shooterMotor1;
    private SparkMax shooterMotor2;

    private RelativeEncoder shooterEncoder1;
    private RelativeEncoder shooterEncoder2;

    private final ProfiledPIDController shooterPID = new ProfiledPIDController(
        Constants.ShooterConstants.SHOOTER_P,
        Constants.ShooterConstants.SHOOTER_I, 
        Constants.ShooterConstants.SHOOTER_D,
        new TrapezoidProfile.Constraints(
            Constants.ShooterConstants.SHOOTER_MAX_ACCELERATION.in(RotationsPerSecondPerSecond),
            Constants.ShooterConstants.SHOOTER_MAX_JERK // Unit is rotations/s^3
        )
    );

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
        Constants.ShooterConstants.SHOOTER_S.in(Volts),
        Constants.ShooterConstants.SHOOTER_V, // Unit is V/(rotations/s)
        Constants.ShooterConstants.SHOOTER_A // Unit is V/(rotations/s^2)
    );

    private double shooterPrevSetpointVelocity = 0;

    public ShooterSubsystem() {
        if (Constants.ShooterConstants.ENABLED) { 
            shooterMotor1 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
            shooterMotor2 = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);

            shooterEncoder1 = shooterMotor1.getEncoder();
            shooterEncoder2 = shooterMotor2.getEncoder();

            double velocityConversionFactor = (1.0 / 60.0) * Constants.ShooterConstants.SHOOTER_GEAR_RATIO; // Convert from RPM to RPS

            shooterConfig1 = new SparkMaxConfig();
            shooterConfig1.idleMode(IdleMode.kCoast);
            shooterConfig1.inverted(Constants.ShooterConstants.SHOOTER_MOTOR_1_INVERTED);
            shooterConfig1.encoder.velocityConversionFactor(velocityConversionFactor);
            shooterMotor1.configure(shooterConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            shooterConfig2 = new SparkMaxConfig();
            shooterConfig2.idleMode(IdleMode.kCoast);
            shooterConfig2.inverted(Constants.ShooterConstants.SHOOTER_MOTOR_2_INVERTED);
            shooterConfig2.encoder.velocityConversionFactor(velocityConversionFactor); // One of these might need to be inverted
            shooterMotor2.configure(shooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public void setTargetSpeed(AngularVelocity speed) {
        shooterPID.setGoal(speed.in(RotationsPerSecond));

        SmartDashboard.putNumber("Shooter/Target Speed", speed.in(RotationsPerSecond) * 60.0);
    }

    public AngularVelocity getTargetSpeed() {
        return RotationsPerSecond.of(shooterPID.getGoal().position);
    }

    public AngularVelocity getSpeed() {
        if (Constants.ShooterConstants.ENABLED == false) {
            return RotationsPerSecond.of(0);
        } 
        
        double speed1 = shooterEncoder1.getVelocity();
        double speed2 = shooterEncoder2.getVelocity();

        return RotationsPerSecond.of((speed1 + speed2) / 2.0);
    }

    public void resetShooter() {
        double currentSpeed = getSpeed().in(RotationsPerSecond);
        shooterPID.reset(currentSpeed);
        shooterPrevSetpointVelocity = currentSpeed;
    }

    @Override
    public void periodic() {
        if (Constants.ShooterConstants.ENABLED) {
            double shooterVoltage = shooterPID.calculate(getSpeed().in(RotationsPerSecond));
            TrapezoidProfile.State shooterState = shooterPID.getSetpoint();
            shooterVoltage += shooterFF.calculateWithVelocities(
                shooterPrevSetpointVelocity,
                shooterState.position
            );
            shooterVoltage = MathUtil.clamp(
                shooterVoltage, 
                -10.0, 
            10.0
            );
            shooterMotor1.setVoltage(shooterVoltage);
            shooterMotor2.setVoltage(shooterVoltage);

            shooterPrevSetpointVelocity = shooterState.position;

            SmartDashboard.putNumber("Shooter/Motor Voltage", shooterVoltage);

            SmartDashboard.putNumber("Shooter/Motor 1 Speed", shooterEncoder1.getVelocity() * 60.0);
            SmartDashboard.putNumber("Shooter/Motor 2 Speed", shooterEncoder2.getVelocity() * 60.0);
        }  
    }
}
