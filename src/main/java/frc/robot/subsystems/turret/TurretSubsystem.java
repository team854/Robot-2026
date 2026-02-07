package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.HootReplay.SignalData;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax turretYawMotor = new SparkMax(Constants.TurretConstants.TURRET_YAW_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder turretYawEncoder = turretYawMotor.getEncoder();
    private final SparkMaxConfig turretYawConfig = new SparkMaxConfig();
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
        Constants.TurretConstants.TURRET_YAW_V.in(Volts), // Unit is V/(rad/s)
        Constants.TurretConstants.TURRET_YAW_A.in(Volts) // Unit is V/(rad/s^2)
    );
    private double turretPrevYawSetpointVelocity = 0;


    private final SparkMax turretPitchMotor = new SparkMax(Constants.TurretConstants.TURRET_PITCH_MOTOR_ID, MotorType.kBrushless);
    private final AbsoluteEncoder turretPitchAbsoluteEncoder = turretPitchMotor.getAbsoluteEncoder();
    private final SparkMaxConfig turretPitchConfig = new SparkMaxConfig();
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
        Constants.TurretConstants.TURRET_PITCH_V.in(Volts), // Unit is V/(rad/s)
        Constants.TurretConstants.TURRET_PITCH_A.in(Volts) // Unit is V/(rad/s^2)
    );
    private double turretPrevPitchSetpointVelocity = 0;

    public TurretSubsystem() {
        // Configure motors
        turretYawConfig.inverted(Constants.TurretConstants.TURRET_YAW_MOTOR_INVERTED);
        double yawConverstionFactor = (2.0 * Math.PI) / Constants.TurretConstants.TURRET_YAW_GEAR_RATIO;
        turretYawConfig.encoder.positionConversionFactor(yawConverstionFactor);
        turretYawConfig.encoder.velocityConversionFactor(yawConverstionFactor / 60.0);
        turretYawMotor.configure(turretYawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turretPitchConfig.inverted(Constants.TurretConstants.TURRET_PITCH_MOTOR_INVERTED);
        double pitchConverstionFactor = (2.0 * Math.PI);
        turretPitchConfig.absoluteEncoder.positionConversionFactor(pitchConverstionFactor);
        turretPitchConfig.absoluteEncoder.velocityConversionFactor(pitchConverstionFactor / 60);
        turretPitchConfig.absoluteEncoder.inverted(Constants.TurretConstants.TURRET_PITCH_ENCODER_INVERTED);
        turretPitchConfig.absoluteEncoder.zeroOffset(Constants.TurretConstants.TURRET_PITCH_ZERO_OFFSET);
        turretPitchMotor.configure(turretPitchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTurretYaw(Angle angle) {
        turretYawPID.setGoal(
            MathUtil.clamp(angle.in(Radian), Constants.TurretConstants.TURRET_YAW_LOWER_LIMIT.in(Radian), Constants.TurretConstants.TURRET_YAW_UPPER_LIMIT.in(Radian))
        );

        SmartDashboard.putNumber("Turret/Target Yaw", turretYawPID.getGoal().position * (180 / Math.PI));
    }

    public void setTurretPitch(Angle angle) {
        turretPitchPID.setGoal(
            MathUtil.clamp(angle.in(Radian), Constants.TurretConstants.TURRET_LOWER_LIMIT.in(Radian), Constants.TurretConstants.TURRET_UPPER_LIMIT.in(Radian))
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
        return Radian.of(turretYawEncoder.getPosition());
    }

    public Angle getTurretPitch() {
        return Radian.of(turretPitchAbsoluteEncoder.getPosition());
    }

    public Angle getTurretPointAngle(Angle globalAngle) {
        return Radian.of(
            MathUtil.angleModulus(globalAngle.in(Radian) - RobotContainer.swerveSubsystem.getSwerveDrive().getPose().getRotation().getRadians())
        );
    }

    @Override
    public void periodic() {

        double turretYawVoltage = turretYawPID.calculate(turretYawEncoder.getPosition());
        TrapezoidProfile.State turretYawState = turretYawPID.getSetpoint();
        turretYawVoltage += turretYawFF.calculateWithVelocities(turretPrevYawSetpointVelocity, turretYawState.velocity);
        turretYawVoltage = MathUtil.clamp(
            turretYawVoltage, 
           -10.0, 
           10.0
        );
        turretYawMotor.setVoltage(turretYawVoltage);
        turretPrevYawSetpointVelocity = turretYawState.velocity;

        double turretPitchVoltage = turretPitchPID.calculate(turretPitchAbsoluteEncoder.getPosition());
        TrapezoidProfile.State turretPitchState = turretPitchPID.getSetpoint();
        turretPitchVoltage += turretPitchFF.calculateWithVelocities(turretPitchState.position, turretPrevPitchSetpointVelocity, turretPitchState.velocity);
        turretPitchVoltage = MathUtil.clamp(
            turretPitchVoltage, 
            -10.0, 
            10.0
        );
        turretPitchMotor.setVoltage(turretPitchVoltage);
        turretPrevPitchSetpointVelocity = turretPitchState.velocity;

        SmartDashboard.putNumber("Turret/Yaw Voltage", turretYawVoltage);
        SmartDashboard.putNumber("Turret/Pitch Voltage", turretPitchVoltage);

        SmartDashboard.putNumber("Turret/Current Yaw", turretYawEncoder.getPosition() * (180 / Math.PI));
        SmartDashboard.putNumber("Turret/Current Pitch", turretPitchAbsoluteEncoder.getPosition() * (180 / Math.PI));
    }
}