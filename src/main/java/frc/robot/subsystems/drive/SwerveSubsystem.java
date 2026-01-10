package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import java.io.File;
import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{
    
    private final SwerveDrive swerveDrive;
    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI); //Enables connection to the RIO
    private final Rotation3d gyroOffset = new Rotation3d(
        0.0,
        0.0,
        Constants.SwerveConstants.GYRO_OFFSET.in(Radian)
    );

    public SwerveSubsystem() {
        swervelib.telemetry.SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {

            //Will throw error if no swerve directory (Thats the purpose)
            swerveDrive = new SwerveParser(Constants.SwerveConstants.SWERVE_DIRECTORY)
                .createSwerveDrive(
                    Constants.SwerveConstants.MAX_SPEED.in(MetersPerSecond),
                    new Pose2d(
                        new Translation2d(Meter.of(1), Meter.of(4)),
                        Rotation2d.fromDegrees(0)));
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setGyroOffset(gyroOffset);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
        System.out.println("Zeroed Swerve");
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        swerveDrive.driveFieldOriented(speeds);
    }

    public Angle getNavXAngle() {
        return Degree.of(navx.getAngle());
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> speeds) {
        return run(() -> swerveDrive.driveFieldOriented(speeds.get()));
    }
}
