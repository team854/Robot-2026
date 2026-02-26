package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.libraries.SubsystemStateMachine;

public class ShooterSubsystem extends SubsystemStateMachine<frc.robot.subsystems.turret.ShooterSubsystem.ShooterState> {
    private final double SHOOTER_THRESHOLD = 1; // RPS

    public enum ShooterState {
        IDLE,
        SPOOLING,
        READY
    }

    private final ShooterIO io;

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

    public ShooterSubsystem(ShooterIO io) {
        super(ShooterState.IDLE);

        this.io = io;
    }

    public void setTargetSpeed(AngularVelocity speed) {
        shooterPID.setGoal(speed.in(RotationsPerSecond));

        SmartDashboard.putNumber("Shooter/Target Speed", speed.in(RotationsPerSecond) * 60.0);
    }

    public AngularVelocity getTargetSpeed() {
        return RotationsPerSecond.of(shooterPID.getGoal().position);
    }

    public AngularVelocity getSpeed() {
        return RotationsPerSecond.of((io.getMotor1RPS() + io.getMotor2RPS()) / 2.0);
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
        // Safety Check as the desired state should only ever IDLE or READY
        if (getDesiredState() == ShooterState.SPOOLING) {
            setDesiredState(ShooterState.IDLE);
        }
        
        
        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == ShooterState.READY) {
                    resetShooter();
                    transitionTo(ShooterState.SPOOLING);
                }
                break;
            case SPOOLING:
                if (getDesiredState() == ShooterState.IDLE) {
                    transitionTo(ShooterState.IDLE);
                } else if (Math.abs(getSpeed().in(RotationsPerSecond) - getTargetSpeed().in(RotationsPerSecond)) < SHOOTER_THRESHOLD) {
                    transitionTo(ShooterState.READY);
                }

                break;
            case READY:
                if (getDesiredState() == ShooterState.IDLE) {
                    transitionTo(ShooterState.IDLE);
                } else if (Math.abs(getSpeed().in(RotationsPerSecond) - getTargetSpeed().in(RotationsPerSecond)) > (SHOOTER_THRESHOLD + 0.1)) {
                    transitionTo(ShooterState.SPOOLING);
                }

                break;
        }

        double shooterVoltage = 0.0;
        switch (getCurrentState()) {
            case IDLE:
                shooterVoltage = 0.0;
                break;
            case SPOOLING:
                shooterVoltage = calculateShooterVoltage();
                break;
            case READY:
                shooterVoltage = calculateShooterVoltage();
                break;
        }

        io.setMotorsVoltage(shooterVoltage);

        SmartDashboard.putNumber("Shooter/Motor Voltage", shooterVoltage);

        SmartDashboard.putNumber("Shooter/Motor 1 Speed", io.getMotor1RPS() * 60.0);
        SmartDashboard.putNumber("Shooter/Motor 2 Speed", io.getMotor2RPS() * 60.0);

        SmartDashboard.putString("Shooter/Current State", getCurrentState().name());
        SmartDashboard.putString("Shooter/Desired State", getDesiredState().name());
    }
}
