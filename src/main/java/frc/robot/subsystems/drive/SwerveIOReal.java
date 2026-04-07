package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveIOReal implements SwerveIO {

    private SwerveDrive swerveDrive;
    private final Rotation3d gyroOffset = new Rotation3d(
        0.0,
        0.0,
        Constants.SwerveConstants.GYRO_OFFSET.in(Radian)
    );

    public SwerveIOReal() {
        swervelib.telemetry.SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {

            // Will throw error if no swerve directory (Thats the purpose)
            swerveDrive = new SwerveParser(Constants.SwerveConstants.SWERVE_DIRECTORY)
                .createSwerveDrive(
                    Constants.SwerveConstants.MAX_SPEED.in(MetersPerSecond),
                    new Pose2d(
                        new Translation2d(Meter.of(3), Meter.of(4)),
                        Rotation2d.fromDegrees(0)));
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.chassisVelocityCorrection = true;

        swerveDrive.setChassisDiscretization(true, 0.02);
        swerveDrive.setGyroOffset(gyroOffset);

        //swerveDrive.stopOdometryThread();
    }

    public void resetOdometry(Pose2d initialPose) {
        swerveDrive.setGyro(new Rotation3d(0, 0, initialPose.getRotation().getRadians()));

        swerveDrive.resetOdometry(initialPose);
        RobotContainer.questNavSubsystem.resetPose(new Pose3d(new Translation3d(initialPose.getTranslation()), new Rotation3d(0, 0, initialPose.getRotation().getRadians())));
    }

    public Pose2d getPose2d() {
        return swerveDrive.getPose();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public ChassisSpeeds getFieldChassisSpeeds() {
        return swerveDrive.getFieldVelocity();
    }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        swerveDrive.driveFieldOriented(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        swerveDrive.setChassisSpeeds(speeds); 
    }

    public double getAngle() {
        return swerveDrive.getYaw().getRadians();
    }

    public double getAngularVelocity() {
        return swerveDrive.getGyro().getYawAngularVelocity().in(RadiansPerSecond);
    }

    public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {
        swerveDrive.addVisionMeasurement(robotPose, timestamp, stdDevs);
    }

    public PIDConstants getHeadingPIDConstants() {
        return new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p, swerveDrive.swerveController.config.headingPIDF.i, swerveDrive.swerveController.config.headingPIDF.d);
    }
    
    @Override
    public boolean checkCANError() {
        for (SwerveModule swerveModule : swerveDrive.getModules()) {
            SparkBase angleMotor = (SparkBase) swerveModule.getAngleMotor().getMotor();
            SparkBase driveMotor = (SparkBase) swerveModule.getDriveMotor().getMotor();

            angleMotor.getBusVoltage();
            if (angleMotor.getFaults().can == true) {
                return true;
            }

            driveMotor.getBusVoltage();
            if (driveMotor.getFaults().can == true) {
                return true;
            }
        }

        return false;
    }

    @Override
    public boolean checkAngleAbsoluteEncodersError() {
        for (SwerveModule swerveModule : swerveDrive.getModules()) {
            if (swerveModule.getAbsoluteEncoderReadIssue()) {
                return true;
            }
        }

        return false;
    }

    @Override
    public boolean checkPigeonError() {
        Pigeon2 pigeon2 = (Pigeon2) swerveDrive.getGyro().getIMU();
        return !pigeon2.isConnected();
    }
}
