package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.SubsystemStateMachine;

public class TurretSubsystem extends SubsystemStateMachine<frc.robot.subsystems.turret.TurretSubsystem.TurretState> {

    private final double TURRET_THRESHOLD = 0.02; // Radian

    public enum TurretState {
        IDLE,
        HOMING,
        STOWED,
        AIMING,
        READY
    }

    private SparkMax turretYawMotor; 
    private RelativeEncoder turretYawEncoder;
    private SparkMaxConfig turretYawConfig;
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


    private SparkMax turretPitchMotor;
    private AbsoluteEncoder turretPitchAbsoluteEncoder;
    private SparkMaxConfig turretPitchConfig;
    private final ProfiledPIDController turretPitchPID = new ProfiledPIDController(
        Constants.TurretConstants.TURRET_PITCH_P,
        Constants.TurretConstants.TURRET_PITCH_I,
        Constants.TurretConstants.TURRET_PITCH_D,
        new TrapezoidProfile.Constraints(
            Constants.TurretConstants.TURRET_PITCH_MAX_VELOCITY.in(RadiansPerSecond),
            Constants.TurretConstants.TURRET_PITCH_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
        )
    );
    private final ArmFeedforward turretPitchFF = new ArmFeedforward(
        Constants.TurretConstants.TURRET_PITCH_S.in(Volts),
        Constants.TurretConstants.TURRET_PITCH_G.in(Volts),
        Constants.TurretConstants.TURRET_PITCH_V, // Unit is V/(rad/s)
        Constants.TurretConstants.TURRET_PITCH_A // Unit is V/(rad/s^2)
    );
    private double turretPrevPitchSetpointVelocity = 0;

    public TurretSubsystem() {
        super(TurretState.IDLE);


        if (Constants.TurretConstants.ENABLED) {
            turretYawMotor = new SparkMax(Constants.TurretConstants.TURRET_YAW_MOTOR_ID, MotorType.kBrushless);
            turretPitchMotor = new SparkMax(Constants.TurretConstants.TURRET_PITCH_MOTOR_ID, MotorType.kBrushless);

            turretYawEncoder = turretYawMotor.getEncoder();
            turretPitchAbsoluteEncoder = turretPitchMotor.getAbsoluteEncoder();

            // Configure motors
            turretYawConfig = new SparkMaxConfig();
            turretYawConfig.inverted(Constants.TurretConstants.TURRET_YAW_MOTOR_INVERTED);
            double yawConverstionFactor = (2.0 * Math.PI) / Constants.TurretConstants.TURRET_YAW_GEAR_RATIO;
            turretYawConfig.encoder.positionConversionFactor(yawConverstionFactor);
            turretYawConfig.encoder.velocityConversionFactor(yawConverstionFactor / 60.0);
            turretYawMotor.configure(turretYawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            turretPitchConfig = new SparkMaxConfig();
            turretPitchConfig.inverted(Constants.TurretConstants.TURRET_PITCH_MOTOR_INVERTED);
            double pitchConverstionFactor = (2.0 * Math.PI);
            turretPitchConfig.absoluteEncoder.positionConversionFactor(pitchConverstionFactor);
            turretPitchConfig.absoluteEncoder.velocityConversionFactor(pitchConverstionFactor / 60.0);
            turretPitchConfig.absoluteEncoder.inverted(Constants.TurretConstants.TURRET_PITCH_ENCODER_INVERTED);
            turretPitchConfig.absoluteEncoder.zeroOffset(Constants.TurretConstants.TURRET_PITCH_ZERO_OFFSET);
            turretPitchMotor.configure(turretPitchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    public void setTurretYaw(Angle angle) {
        turretYawPID.setGoal(
            MathUtil.clamp(angle.in(Radian), Constants.TurretConstants.TURRET_YAW_LOWER_LIMIT.in(Radian), Constants.TurretConstants.TURRET_YAW_UPPER_LIMIT.in(Radian))
        );

        SmartDashboard.putNumber("Turret/Target Yaw", turretYawPID.getGoal().position * (180 / Math.PI));
    }

    public void setTurretPitch(Angle angle) {
        turretPitchPID.setGoal(
            MathUtil.clamp(angle.in(Radian), Constants.TurretConstants.TURRET_PITCH_LOWER_LIMIT.in(Radian), Constants.TurretConstants.TURRET_PITCH_UPPER_LIMIT.in(Radian))
        );

        SmartDashboard.putNumber("Turret/Target Pitch", turretPitchPID.getGoal().position * (180 / Math.PI));
    }

    public Angle getTurretTargetYaw() {
        return Radian.of(turretYawPID.getGoal().position);
    }

    public Angle getTurretTargetPitch() {
        return Radian.of(turretPitchPID.getGoal().position);
    }

    public Angle getTurretYaw() {
        if (Constants.TurretConstants.ENABLED == false) {
            return Radian.of(0);
        }

        return Radian.of(turretYawEncoder.getPosition());
    }

    public Angle getTurretPitch() {
        if (Constants.TurretConstants.ENABLED == false) {
            return Radian.of(0);
        }

        return Radian.of(turretPitchAbsoluteEncoder.getPosition());
    }

    public Angle getTurretPointAngle(Angle globalAngle) {
        return Radian.of(
            MathUtil.angleModulus(globalAngle.in(Radian) - RobotContainer.swerveSubsystem.getPose2d().getRotation().getRadians())
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

    private double[] calculateTurretVoltage() {
        double turretYawVoltage = turretYawPID.calculate(turretYawEncoder.getPosition());
        TrapezoidProfile.State turretYawState = turretYawPID.getSetpoint();
        turretYawVoltage += turretYawFF.calculateWithVelocities(turretPrevYawSetpointVelocity, turretYawState.velocity);
        turretYawVoltage = MathUtil.clamp(
            turretYawVoltage, 
        -10.0, 
        10.0
        );
        turretPrevYawSetpointVelocity = turretYawState.velocity;

        double turretPitchVoltage = turretPitchPID.calculate(turretPitchAbsoluteEncoder.getPosition());
        TrapezoidProfile.State turretPitchState = turretPitchPID.getSetpoint();
        turretPitchVoltage += turretPitchFF.calculateWithVelocities(turretPitchState.position, turretPrevPitchSetpointVelocity, turretPitchState.velocity);
        turretPitchVoltage = MathUtil.clamp(
            turretPitchVoltage, 
            -10.0, 
            10.0
        );
        turretPrevPitchSetpointVelocity = turretPitchState.velocity;

        return new double[]{turretYawVoltage, turretPitchVoltage};
    }

    @Override
    public void periodic() {
        if (Constants.TurretConstants.ENABLED) {

            double[] turretVoltage = new double[]{0, 0};

            // Safety Check as the desired state should only ever IDLE, HOMING, STOWED, or READY
            if (getDesiredState() == TurretState.AIMING) {
                setDesiredState(TurretState.IDLE);
            }

            switch (getCurrentState()) {
                case IDLE:
                    turretVoltage = new double[]{0, 0};

                    if (getDesiredState() == TurretState.READY) {
                        resetTurretPitch();
                        resetTurretYaw();
                        transitionTo(TurretState.AIMING);
                    } else if (getDesiredState() == TurretState.STOWED) {
                        resetTurretPitch();
                        resetTurretYaw();
                        transitionTo(TurretState.STOWED);
                    } else if (getDesiredState() == TurretState.HOMING) {
                        resetTurretPitch();
                        resetTurretYaw();
                        transitionTo(TurretState.HOMING);
                    }

                    break;
                case HOMING:
                    // TODO

                    if (getDesiredState() == TurretState.IDLE) {
                        transitionTo(TurretState.IDLE);
                    } else if (getDesiredState() == TurretState.STOWED) {
                        resetTurretPitch();
                        resetTurretYaw();
                        transitionTo(TurretState.STOWED);
                    } else if (getDesiredState() == TurretState.READY) {
                        resetTurretPitch();
                        resetTurretYaw();
                        transitionTo(TurretState.AIMING);
                    }

                    break;

                case STOWED:   
                    setTurretPitch(Constants.TurretConstants.TURRET_STOWED_PITCH_ANGLE);    
                    turretVoltage = calculateTurretVoltage();

                    if (getDesiredState() == TurretState.IDLE) {
                        transitionTo(TurretState.IDLE);
                    } else if (getDesiredState() == TurretState.HOMING) {
                        transitionTo(TurretState.HOMING);
                    } else if (getDesiredState() == TurretState.READY) {
                        transitionTo(TurretState.AIMING);
                    }
                    break;

                case AIMING:
                    turretVoltage = calculateTurretVoltage();

                    if (getDesiredState() == TurretState.IDLE) {
                        transitionTo(TurretState.IDLE);
                    } else if (getDesiredState() == TurretState.STOWED) {
                        transitionTo(TurretState.STOWED);
                    } else if (getDesiredState() == TurretState.HOMING) {
                        transitionTo(TurretState.HOMING);
                    } else if (
                        Math.abs(getTurretTargetPitch().in(Radian) - getTurretPitch().in(Radian)) <= TURRET_THRESHOLD &&
                        Math.abs(getTurretTargetYaw().in(Radian) - getTurretYaw().in(Radian)) <= TURRET_THRESHOLD
                    ) {
                        transitionTo(TurretState.READY);
                    }

                    break;

                case READY:
                    turretVoltage = calculateTurretVoltage();

                     if (getDesiredState() == TurretState.IDLE) {
                        transitionTo(TurretState.IDLE);
                    } else if (getDesiredState() == TurretState.STOWED) {
                        transitionTo(TurretState.STOWED);
                    } else if (getDesiredState() == TurretState.HOMING) {
                        transitionTo(TurretState.HOMING);
                    } else if (
                        Math.abs(getTurretTargetPitch().in(Radian) - getTurretPitch().in(Radian)) >= TURRET_THRESHOLD ||
                        Math.abs(getTurretTargetYaw().in(Radian) - getTurretYaw().in(Radian)) >= TURRET_THRESHOLD
                    ) {
                        transitionTo(TurretState.AIMING);
                    }

                    break;
            }

            turretYawMotor.setVoltage(turretVoltage[0]);
            turretPitchMotor.setVoltage(turretVoltage[1]);

            SmartDashboard.putNumber("Turret/Yaw Voltage", turretVoltage[0]);
            SmartDashboard.putNumber("Turret/Pitch Voltage", turretVoltage[1]);

            SmartDashboard.putNumber("Turret/Current Yaw", turretYawEncoder.getPosition() * (180 / Math.PI));
            SmartDashboard.putNumber("Turret/Current Pitch", turretPitchAbsoluteEncoder.getPosition() * (180 / Math.PI));

            SmartDashboard.putString("Turret/Current State", getCurrentState().name());
            SmartDashboard.putString("Turret/Desired State", getDesiredState().name());
        }
    }
}