package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.SubsystemStateMachine;

public class ShooterSubsystem extends SubsystemStateMachine<frc.robot.subsystems.turret.ShooterSubsystem.ShooterState> {
    private final double SHOOTER_THRESHOLD = 1; // RPS

    public enum ShooterState {
        IDLE,
        SPOOLING,
        READY
    }

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
        super(ShooterState.IDLE);

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
        if (shooterEncoder1 == null || shooterEncoder2 == null) {
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

    private double calculateShooterVoltage() {
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

        shooterPrevSetpointVelocity = shooterState.position;

        return shooterVoltage;
    }

    @Override
    public void periodic() {
        
        if (Constants.ShooterConstants.ENABLED == false) {
            return;
        }
        // Safety Check as the desired state should only ever IDLE or READY
        if (getDesiredState() == ShooterState.SPOOLING) {
            setDesiredState(ShooterState.IDLE);
        }
        
        double shooterVoltage = 0.0;
        switch (getCurrentState()) {
            case IDLE:
                shooterVoltage = 0.0;

                if (getDesiredState() == ShooterState.READY) {
                    resetShooter();
                    transitionTo(ShooterState.SPOOLING);
                }
                break;
            case SPOOLING:
                shooterVoltage = calculateShooterVoltage();

                if (getDesiredState() == ShooterState.IDLE) {
                    transitionTo(ShooterState.IDLE);
                } else if (Math.abs(getSpeed().in(RotationsPerSecond) - getTargetSpeed().in(RotationsPerSecond)) < SHOOTER_THRESHOLD) {
                    transitionTo(ShooterState.READY);
                }

                break;
            case READY:
                shooterVoltage = calculateShooterVoltage();

                if (getDesiredState() == ShooterState.IDLE) {
                    transitionTo(ShooterState.IDLE);
                } else if (Math.abs(getSpeed().in(RotationsPerSecond) - getTargetSpeed().in(RotationsPerSecond)) > (SHOOTER_THRESHOLD + 0.1)) {
                    transitionTo(ShooterState.SPOOLING);
                }

                break;
        }
        shooterMotor1.setVoltage(shooterVoltage);
        shooterMotor2.setVoltage(shooterVoltage);

        SmartDashboard.putNumber("Shooter/Motor Voltage", shooterVoltage);

        SmartDashboard.putNumber("Shooter/Motor 1 Speed", shooterEncoder1.getVelocity() * 60.0);
        SmartDashboard.putNumber("Shooter/Motor 2 Speed", shooterEncoder2.getVelocity() * 60.0);

        SmartDashboard.putString("Shooter/Current State", getCurrentState().name());
        SmartDashboard.putString("Shooter/Desired State", getDesiredState().name());
    }
}
