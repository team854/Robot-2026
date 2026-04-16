package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ErrorConstants;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{
    
    

    private final SwerveIO io;

    private double lastCANErrorTimestamp = Double.NEGATIVE_INFINITY;
    private double lastAbsoluteEncoderErrorTimestamp = Double.NEGATIVE_INFINITY;

    public SwerveSubsystem(SwerveIO io) {
        this.io = io;

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config", true);
            e.printStackTrace();
            return;
        }
        
        AutoBuilder.configure(
            this::getPose2d,
            this::resetOdometry,
            this::getRobotChassisSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(Constants.SwerveConstants.DRIVE_P, Constants.SwerveConstants.DRIVE_I, Constants.SwerveConstants.DRIVE_D),
                io.getHeadingPIDConstants()
            ),
            config, 
            RobotContainer::isRedAlliance,
            this
        );
    }

    public void resetOdometry(Pose2d initialPose) {
        io.resetOdometry(initialPose);
    }

    public void resetOdometry(Translation2d initialTranslation) {
        io.resetOdometry(
            new Pose2d(
                initialTranslation,
                Rotation2d.fromDegrees(RobotContainer.isBlueAlliance() ? 180 : 0)
            )
        );
    }

    public void zeroGyro() {
        resetOdometry(io.getPose2d().getTranslation());
        System.out.println("Zeroed Swerve");
    }

    public Command zeroGyroCommand() {

        return Commands.runOnce(() -> {zeroGyro();});
    }

    public Pose2d getPose2d() {
        return io.getPose2d();
    }

     public SwerveDrive getSwerveDrive() {
        return io.getSwerveDrive();
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        return io.getRobotChassisSpeeds();
    }

    public ChassisSpeeds getFieldChassisSpeeds() {
        return io.getFieldChassisSpeeds();
    }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        io.driveFieldOriented(speeds);
    }


    public void driveRobotRelative(ChassisSpeeds speeds) {
        io.driveRobotRelative(speeds);
    }


    public Angle getAngle() {
        return Radian.of(io.getAngle());
    }

    public AngularVelocity getAngularVelocity() {
        return RadiansPerSecond.of(io.getAngularVelocity());
    }

    public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {
        io.addVisionMeasurement(robotPose, timestamp, stdDevs);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> speeds) {
        return run(() -> io.driveFieldOriented(speeds.get()));
    }

    public void checkCanHealth() {
        double timestamp = Timer.getFPGATimestamp();
        if (io.checkCANError()) {
            lastCANErrorTimestamp = timestamp;
        }

        if ((timestamp - lastCANErrorTimestamp) < Constants.HealthConstants.CAN_ERROR_PERSIST.in(Second)) {
            RobotContainer.healthSubsystem.reportError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        } else {
            RobotContainer.healthSubsystem.clearError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        }
    }

    public void checkAngleAbsoluteEncodersHealth() {
        double timestamp = Timer.getFPGATimestamp();
        if (io.checkAngleAbsoluteEncodersError()) {
            lastAbsoluteEncoderErrorTimestamp = timestamp;
        }

        if ((timestamp - lastAbsoluteEncoderErrorTimestamp) < Constants.HealthConstants.ABSOLUTE_ENCODER_ERROR_PERSIST.in(Second)) {
            RobotContainer.healthSubsystem.reportError(getSubsystem(), ErrorConstants.SWERVE_ABSOLUTE_ENCODER_ERROR);
        } else {
            RobotContainer.healthSubsystem.clearError(getSubsystem(), ErrorConstants.SWERVE_ABSOLUTE_ENCODER_ERROR);
        }
    }

    @Override
    public void periodic() {
        checkCanHealth();
        checkAngleAbsoluteEncodersHealth();
    }
}