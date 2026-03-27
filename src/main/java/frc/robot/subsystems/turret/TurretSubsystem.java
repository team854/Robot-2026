package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.SubsystemStateMachine;
import frc.robot.subsystems.turret.CalculationSubsystem.Zone;

public class TurretSubsystem extends SubsystemStateMachine<frc.robot.subsystems.turret.TurretSubsystem.TurretState> {

    private final double TURRET_THRESHOLD = 0.02; // Radian

    public enum TurretState {
        IDLE,
        HOMING,
        STOWED,
        AIMING,
        READY,
        MANUAL
    }

    private enum HomingStage {
        SEARCHING,
        REFINING_START,
        REFINING_END
    }

    private final TurretIO io;

    
    private final ProfiledPIDController turretYawPID = new ProfiledPIDController(
        Constants.TurretConstants.TURRET_YAW_P,
        Constants.TurretConstants.TURRET_YAW_I,
        Constants.TurretConstants.TURRET_YAW_D,
        new TrapezoidProfile.Constraints(
            Constants.TurretConstants.TURRET_YAW_MAX_VELOCITY.in(RadiansPerSecond),
            Constants.TurretConstants.TURRET_YAW_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
        )
    );
    private final SimpleMotorFeedforward turretYawFF = new SimpleMotorFeedforward(
        Constants.TurretConstants.TURRET_YAW_S.in(Volts),
        Constants.TurretConstants.TURRET_YAW_V, // Unit is V/(rad/s)
        Constants.TurretConstants.TURRET_YAW_A // Unit is V/(rad/s^2)
    );
    private double turretPrevYawSetpointVelocity = 0;

    private final ProfiledPIDController turretPitchPID = new ProfiledPIDController(
        Constants.TurretConstants.TURRET_PITCH_P,
        Constants.TurretConstants.TURRET_PITCH_I,
        Constants.TurretConstants.TURRET_PITCH_D,
        new TrapezoidProfile.Constraints(
            Constants.TurretConstants.TURRET_PITCH_MAX_VELOCITY.in(RadiansPerSecond),
            Constants.TurretConstants.TURRET_PITCH_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
        )
    );
    private final SimpleMotorFeedforward turretPitchFF = new SimpleMotorFeedforward(
        Constants.TurretConstants.TURRET_PITCH_S.in(Volts),
        Constants.TurretConstants.TURRET_PITCH_V, // Unit is V/(rad/s)
        Constants.TurretConstants.TURRET_PITCH_A // Unit is V/(rad/s^2)
    );
    private double turretPrevPitchSetpointVelocity = 0;

    private Angle turretManualYaw = Degree.of(0);
    private Angle turretManualPitch = Degree.of(0);

    private double turretStowedYawAngle = 0;

    private HomingStage turretHomingStage = HomingStage.SEARCHING;
    private double turretHomingStart = 0;

    public TurretSubsystem(TurretIO io) {
        super(TurretState.IDLE, TurretState.IDLE);

        this.io = io;

        turretYawPID.setIZone(Constants.TurretConstants.TURRET_YAW_IZONE.in(Radian));
        turretPitchPID.setIZone(Constants.TurretConstants.TURRET_PITCH_IZONE.in(Radian));
    }

    public void setTurretYaw(Angle angle) {
        double clampedAngle = MathUtil.clamp(angle.in(Radian), Constants.TurretConstants.TURRET_YAW_LOWER_LIMIT.in(Radian), Constants.TurretConstants.TURRET_YAW_UPPER_LIMIT.in(Radian));

        double targetVelocity = 0;
        double robotVelocity = RobotContainer.swerveSubsystem.getAngularVelocity().in(RadiansPerSecond);
        if (angle.in(Radian) > Constants.TurretConstants.TURRET_YAW_LOWER_LIMIT.in(Radian)
            && angle.in(Radian) < Constants.TurretConstants.TURRET_YAW_UPPER_LIMIT.in(Radian) 
            && Math.abs(robotVelocity) > 0.01) {
            targetVelocity = robotVelocity;
        }

        turretYawPID.setGoal(
            new TrapezoidProfile.State(clampedAngle, targetVelocity)
        );
    }

    public void setTurretPitch(Angle angle) {
        turretPitchPID.setGoal(
            MathUtil.clamp(
                angle.in(Radian), 
                Constants.TurretConstants.TURRET_PITCH_LOWER_LIMIT.in(Radian), 
                Constants.TurretConstants.TURRET_PITCH_UPPER_LIMIT.in(Radian)
            )
        );
    }

    public Angle getTurretTargetYaw() {
        return Radian.of(turretYawPID.getGoal().position);
    }

    public Angle getTurretTargetPitch() {
        return Radian.of(turretPitchPID.getGoal().position);
    }

    public Angle getTurretYaw() {
        return Radian.of(io.getYawRadians());
    }

    public Angle getTurretPitch() {
        return Radian.of(io.getPitchRadians());
    }

    public Angle getTurretPointAngle(Angle globalAngle) {
        return Radian.of(
            MathUtil.angleModulus(RobotContainer.swerveSubsystem.getPose2d().getRotation().getRadians() - globalAngle.in(Radian) + Math.PI)
        );
    }

    public void resetTurretPitch() {
        double currentPitch = getTurretPitch().in(Radian);
        turretPitchPID.reset(currentPitch);
        turretPrevPitchSetpointVelocity = 0;
    }

    public void resetTurretYaw() {
        double currentYaw = getTurretYaw().in(Radian);
        turretYawPID.reset(currentYaw);
        turretPrevYawSetpointVelocity = 0;
    }

    private void resetTurretHoming() {
        turretHomingStage = HomingStage.SEARCHING;
        turretHomingStart = 0;
        io.resetHomingCounter();
    }

    private double calculateTurretYawVoltage() {
        double turretYawVoltage = MathUtil.clamp(
            turretYawPID.calculate(io.getYawRadians()) * 1.5,
            -0.75,
            0.75
        );
        
        //turretYawVoltage = 0;

        TrapezoidProfile.State turretYawState = turretYawPID.getSetpoint();
        turretYawVoltage += turretYawFF.calculateWithVelocities(turretPrevYawSetpointVelocity, turretYawState.velocity);

        
        turretYawVoltage = MathUtil.clamp(
            turretYawVoltage, 
            -10.0, 
            10.0
        );
        turretPrevYawSetpointVelocity = turretYawState.velocity;

        return turretYawVoltage;
    }

    private double calculateTurretPitchVoltage() {
        double turretPitchVoltage = MathUtil.clamp(
            turretPitchPID.calculate(io.getPitchRadians()),
            -0.3,
            0.3
        );
        
        TrapezoidProfile.State turretPitchState = turretPitchPID.getSetpoint();
        turretPitchVoltage += turretPitchFF.calculateWithVelocities(turretPrevPitchSetpointVelocity, turretPitchState.velocity);
        turretPitchVoltage += Math.cos(turretPitchState.position) * Constants.TurretConstants.TURRET_PITCH_G.in(Volt);
        turretPitchVoltage = MathUtil.clamp(
            turretPitchVoltage, 
            -10.0, 
        10.0
        );
        turretPrevPitchSetpointVelocity = turretPitchState.velocity;

        return turretPitchVoltage;
    }

    public void setOverrideAngles(Angle yaw, Angle pitch) {
        turretManualYaw = yaw;
        turretManualPitch = pitch;
    }

    @Override
    public void periodic() {
        if (RobotContainer.calculationSubsystem.getZone() == Zone.TRENCH) {
            requestDesiredState(TurretState.STOWED, 30);
        } else {
            requestDesiredState(TurretState.IDLE, 0);
        }

        // Safety Check as the desired state should only ever IDLE, HOMING, STOWED, READY, or MANUAL
        if (getDesiredState() == TurretState.AIMING) {
            requestDesiredState(TurretState.IDLE, 6);
        }

        updateDesiredState();

        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == TurretState.READY) {
                    resetTurretPitch();
                    resetTurretYaw();
                    transitionTo(TurretState.AIMING);
                } else if (getDesiredState() == TurretState.STOWED) {
                    turretStowedYawAngle = io.getYawRadians();
                    resetTurretPitch();
                    resetTurretYaw();
                    transitionTo(TurretState.STOWED);
                } else if (getDesiredState() == TurretState.HOMING) {
                    resetTurretPitch();
                    resetTurretYaw();
                    resetTurretHoming();
                    transitionTo(TurretState.HOMING);
                }

                break;
            case HOMING:
                if (io.getHomingSensor() == false && turretHomingStage == HomingStage.REFINING_END) {
                    io.setYawEncoderPosition((io.getYawRadians() - turretHomingStart + Constants.TurretConstants.TURRET_YAW_OFFSET.in(Radian)));
                    resetTurretPitch();
                    resetTurretYaw();
                    requestDesiredState(TurretState.IDLE, 6);
                    transitionTo(TurretState.IDLE);
                    
                }

                break;

            case STOWED:
                if (getDesiredState() == TurretState.IDLE) {
                    transitionTo(TurretState.IDLE);
                } else if (getDesiredState() == TurretState.HOMING) {
                    resetTurretHoming();
                    transitionTo(TurretState.HOMING);
                } else if (getDesiredState() == TurretState.READY) {
                    transitionTo(TurretState.AIMING);
                }
                break;

            case AIMING:
                if (getDesiredState() == TurretState.IDLE) {
                    transitionTo(TurretState.IDLE);
                } else if (getDesiredState() == TurretState.STOWED) {
                    turretStowedYawAngle = io.getYawRadians();
                    transitionTo(TurretState.STOWED);
                } else if (getDesiredState() == TurretState.HOMING) {
                    resetTurretHoming();
                    transitionTo(TurretState.HOMING);
                } else if (
                    Math.abs(getTurretTargetPitch().in(Radian) - getTurretPitch().in(Radian)) <= TURRET_THRESHOLD &&
                    Math.abs(getTurretTargetYaw().in(Radian) - getTurretYaw().in(Radian)) <= TURRET_THRESHOLD
                ) {
                    transitionTo(TurretState.READY);
                }

                break;

            case READY:
                if (getDesiredState() == TurretState.IDLE) {
                    transitionTo(TurretState.IDLE);
                } else if (getDesiredState() == TurretState.STOWED) {
                    turretStowedYawAngle = io.getYawRadians();
                    transitionTo(TurretState.STOWED);
                } else if (getDesiredState() == TurretState.HOMING) {
                    resetTurretHoming();
                    transitionTo(TurretState.HOMING);
                } else if (
                    Math.abs(getTurretTargetPitch().in(Radian) - getTurretPitch().in(Radian)) >= (TURRET_THRESHOLD + 0.02) ||
                    Math.abs(getTurretTargetYaw().in(Radian) - getTurretYaw().in(Radian)) >= (TURRET_THRESHOLD + 0.02)
                ) {
                    transitionTo(TurretState.AIMING);
                }

                break;

            case MANUAL:
                if (getDesiredState() == TurretState.IDLE) {
                    transitionTo(TurretState.IDLE);
                } else if (getDesiredState() == TurretState.STOWED) {
                    turretStowedYawAngle = io.getYawRadians();
                    resetTurretPitch();
                    resetTurretYaw();
                    transitionTo(TurretState.STOWED);
                } else if (getDesiredState() == TurretState.HOMING) {
                    resetTurretPitch();
                    resetTurretYaw();
                    resetTurretHoming();
                    transitionTo(TurretState.HOMING);
                } else if (getDesiredState() == TurretState.READY) {
                    resetTurretPitch();
                    resetTurretYaw();
                    transitionTo(TurretState.AIMING);
                }
                break;
        }

        double turretYawVoltage = 0.0;
        double turretPitchVoltage = 0.0;
        switch (getCurrentState()) {
            case IDLE:
                turretYawVoltage = 0.0;
                turretPitchVoltage = 0.0;
                break;
                
            case HOMING:
                if (turretHomingStage == HomingStage.SEARCHING) {
                    turretYawVoltage = Constants.TurretConstants.TURRET_YAW_HOMING_SEARCHING_VOLTAGE.in(Volt);
                    if (io.getHomingCounter()) {
                        turretHomingStage = HomingStage.REFINING_START;
                    }
                } else {
                    turretYawVoltage = Constants.TurretConstants.TURRET_YAW_HOMING_REFINING_VOLTAGE.in(Volt);
                    if (io.getHomingSensor() && turretHomingStage == HomingStage.REFINING_START) {
                        turretHomingStart = io.getYawRadians();
                        turretHomingStage = HomingStage.REFINING_END;
                    }
                }
                
                turretPitchVoltage = 0.0;
                
                break;

            case STOWED:
                // Only stow pitch as that is the only attribute that affects height
                setTurretPitch(Constants.TurretConstants.TURRET_STOWED_PITCH_ANGLE);
                setTurretYaw(Radian.of(turretStowedYawAngle));

                turretYawVoltage = calculateTurretYawVoltage();
                turretPitchVoltage = calculateTurretPitchVoltage();
                break;
            case AIMING:
                turretYawVoltage = calculateTurretYawVoltage();
                turretPitchVoltage = calculateTurretPitchVoltage();
                break;
            case READY:
                turretYawVoltage = calculateTurretYawVoltage();
                turretPitchVoltage = calculateTurretPitchVoltage();
                break;
            case MANUAL:
                setTurretYaw(turretManualYaw);
                setTurretPitch(turretManualPitch);
                turretYawVoltage = calculateTurretYawVoltage();
                turretPitchVoltage = calculateTurretPitchVoltage();
                break;

        }

        turretYawVoltage = MathUtil.clamp(turretYawVoltage, -5, 5);
        turretPitchVoltage = MathUtil.clamp(turretPitchVoltage, -5, 5);

        io.setYawMotorVoltage(turretYawVoltage);
        io.setPitchMotorVoltage(turretPitchVoltage);

        SmartDashboard.putNumber("Turret/Yaw Voltage", turretYawVoltage);
        SmartDashboard.putNumber("Turret/Pitch Voltage", turretPitchVoltage);

        SmartDashboard.putNumber("Turret/Current Yaw", io.getYawRadians() * (180 / Math.PI));
        SmartDashboard.putNumber("Turret/Current Pitch", io.getPitchRadians() * (180 / Math.PI));

        SmartDashboard.putNumber("Turret/Current Raw Yaw", io.getRawYawRadians() * (180 / Math.PI));
        SmartDashboard.putNumber("Turret/Current Raw Absolute Yaw", io.getRawAbsoluteYawRadians() * (180 / Math.PI));

        SmartDashboard.putString("Turret/Current State", getCurrentState().name());
        SmartDashboard.putString("Turret/Desired State", getDesiredState().name());

        SmartDashboard.putNumber("Turret/Target Yaw", turretYawPID.getGoal().position * (180 / Math.PI));
        SmartDashboard.putNumber("Turret/Target Pitch", turretPitchPID.getGoal().position * (180 / Math.PI));

        SmartDashboard.putBoolean("Turret/Homing Sensor", io.getHomingSensor());
        SmartDashboard.putBoolean("Turret/Homing Counter", io.getHomingCounter());
    }
}