package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ErrorConstants;
import frc.robot.RobotContainer;
import frc.robot.libraries.SubsystemStateMachine;

public class ShooterSubsystem extends SubsystemStateMachine<frc.robot.subsystems.turret.ShooterSubsystem.ShooterState> {

    public enum ShooterState {
        IDLE,
        SPOOLING,
        READY
    }

    private final ShooterIO io;

    private double shooterTargetVelocity = 0;

    private double lastErrorTimestamp = Double.NEGATIVE_INFINITY;

    public ShooterSubsystem(ShooterIO io) {
        super(ShooterState.IDLE, null);

        if (io == null) {
            throw new IllegalArgumentException("ShooterIO cannot be null");
        }
        
        this.io = io;
    }

    public void setTargetSpeed(AngularVelocity speed) {
        shooterTargetVelocity = MathUtil.clamp(speed.in(RotationsPerSecond), Constants.ShooterConstants.SHOOTER_MIN_VELOCITY.in(RotationsPerSecond), Constants.ShooterConstants.SHOOTER_MAX_VELOCITY.in(RotationsPerSecond));
    }

    public AngularVelocity getTargetSpeed() {
        return RotationsPerSecond.of(shooterTargetVelocity);
    }

    public AngularVelocity getSpeed() {
        return RotationsPerSecond.of((io.getMotor1RPS() + io.getMotor2RPS()) / 2.0);
    }

    public void resetShooter() {
        shooterTargetVelocity = 0;
    }

    public void checkCanHealth() {
        double timestamp = Timer.getFPGATimestamp();
        if (io.checkCANError()) {
            lastErrorTimestamp = timestamp;
        }

        if ((timestamp - lastErrorTimestamp) < Constants.HealthConstants.CAN_ERROR_PERSIST.in(Second)) {
            RobotContainer.healthSubsystem.reportError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        } else {
            RobotContainer.healthSubsystem.clearError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        }
    }

    @Override
    public void statePeriodicBefore() {
        // Safety Check as the desired state should only ever IDLE or READY
        if (getDesiredState() == ShooterState.SPOOLING) {
            requestDesiredState(ShooterState.IDLE, 25);
        }
    }

    @Override
    public void statePeriodic() {
        
        
        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == ShooterState.READY) {
                    transitionTo(ShooterState.SPOOLING);
                }
                break;
            case SPOOLING:
                if (getDesiredState() == ShooterState.IDLE) {
                    transitionTo(ShooterState.IDLE);
                } else if (Math.abs(getSpeed().in(RotationsPerSecond) - getTargetSpeed().in(RotationsPerSecond)) < Constants.ShooterConstants.SHOOTER_READY_THRESHOLD.in(RotationsPerSecond)) {
                    transitionTo(ShooterState.READY);
                }

                break;
            case READY:
                if (getDesiredState() == ShooterState.IDLE) {
                    transitionTo(ShooterState.IDLE);
                } else if (Math.abs(getSpeed().in(RotationsPerSecond) - getTargetSpeed().in(RotationsPerSecond)) > (Constants.ShooterConstants.SHOOTER_READY_THRESHOLD.in(RotationsPerSecond) + 0.1)) {
                    transitionTo(ShooterState.SPOOLING);
                }

                break;
        }


        switch (getCurrentState()) {
            case IDLE:
                io.setMotorsVoltage(0.0);
                break;
            case SPOOLING:
                io.setClosedVelocity(shooterTargetVelocity);
                break;
            case READY:
                io.setClosedVelocity(shooterTargetVelocity);
                break;
            default:
                io.setMotorsVoltage(0.0); 
                System.err.println("Shooter in unknown state: " + getCurrentState());
                break;
        }

        checkCanHealth();

        //SmartDashboard.putNumber("Shooter/Motor Voltage", shooterVoltage);

        SmartDashboard.putNumber("Shooter/Motor 1 Speed", io.getMotor1RPS() * 60.0);
        SmartDashboard.putNumber("Shooter/Motor 2 Speed", io.getMotor2RPS() * 60.0);

        SmartDashboard.putNumber("Shooter/Target Speed", getTargetSpeed().in(RotationsPerSecond) * 60);

        SmartDashboard.putString("Shooter/Current State", getCurrentState().name());
        SmartDashboard.putString("Shooter/Desired State", getDesiredState().name());
    }
}
