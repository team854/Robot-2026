package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{
    
    private SwerveDrive swerveDrive;
    private final Rotation3d gyroOffset = new Rotation3d(
        0.0,
        0.0,
        Constants.SwerveConstants.GYRO_OFFSET.in(Radian)
    );

    public SwerveSubsystem() {
        if (Constants.SwerveConstants.ENABLED) {
            swervelib.telemetry.SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

            try {

                // Will throw error if no swerve directory (Thats the purpose)
                swerveDrive = new SwerveParser(Constants.SwerveConstants.SWERVE_DIRECTORY)
                    .createSwerveDrive(
                        Constants.SwerveConstants.MAX_SPEED.in(MetersPerSecond),
                        new Pose2d(
                            new Translation2d(Meter.of(2), Meter.of(4)),
                            Rotation2d.fromDegrees(0)));
            }
            catch (Exception e) {
                throw new RuntimeException(e);
            }

            swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
            swerveDrive.setGyroOffset(gyroOffset);
        }
    }

    public void zeroGyro() {
        if (Constants.SwerveConstants.ENABLED) {
            swerveDrive.zeroGyro();
            System.out.println("Zeroed Swerve");
        }
    }

    public Pose2d getPose2d() {
        if (Constants.SwerveConstants.ENABLED == false) {
            return new Pose2d();
        }

        return swerveDrive.getPose();
    }

     public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public ChassisSpeeds getChassisSpeeds() {
        if (Constants.SwerveConstants.ENABLED == false) {
            return new ChassisSpeeds();
        }

        return swerveDrive.getRobotVelocity();
    }

    public ChassisSpeeds getFieldChassisSpeeds() {
        if (Constants.SwerveConstants.ENABLED == false) {
            return new ChassisSpeeds();
        }

        return swerveDrive.getFieldVelocity();
    }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        if (Constants.SwerveConstants.ENABLED == false) {
            return;
        }

        swerveDrive.driveFieldOriented(speeds);
    }

    public Angle getNavXAngle() {
        if (Constants.SwerveConstants.ENABLED == false) {
            return Degree.of(0);
        }

        return Degree.of(swerveDrive.getYaw().getDegrees());
    }

    public AngularVelocity getNavXVelocity() {
        if (Constants.SwerveConstants.ENABLED == false) {
            return RadiansPerSecond.of(0);
        }

        return RadiansPerSecond.of(swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
    }
    
    public void addVisionMeasurement(Pose2d robotPose, double timestamp) {
        if (Constants.SwerveConstants.ENABLED == false) {
            return;
        }

        swerveDrive.addVisionMeasurement(robotPose, timestamp);
    }

    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        if (Constants.SwerveConstants.ENABLED == false) {
            return;
        }

        swerveDrive.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> speeds) {
        if (Constants.SwerveConstants.ENABLED == false) {
            return run(() -> {});
        }
        
        return run(() -> swerveDrive.driveFieldOriented(speeds.get()));
    }

    @Override
    public void periodic() {
        
    }
}
