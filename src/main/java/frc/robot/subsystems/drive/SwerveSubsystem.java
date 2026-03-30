package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.time.Instant;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.FieldHelpers;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
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
                    new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p, swerveDrive.swerveController.config.headingPIDF.i, swerveDrive.swerveController.config.headingPIDF.d)
                ),
                config, 
                RobotContainer::isRedAlliance,
                this
            );

            //swerveDrive.stopOdometryThread();
        }
    }

    public void resetOdometry(Pose2d initialPose) {
        if (Constants.SwerveConstants.ENABLED) {
            swerveDrive.resetOdometry(initialPose);
        }
    }

    public void resetOdometry(Translation2d initialTranslation) {
        resetOdometry(
            new Pose2d(
                initialTranslation,
                Rotation2d.fromDegrees(RobotContainer.isBlueAlliance() ? 180 : 0)
            )
        );
    }

    public void zeroGyro() {
        if (Constants.SwerveConstants.ENABLED) {

            resetOdometry(
                new Pose2d(
                    swerveDrive.getPose().getTranslation(),
                    Rotation2d.fromDegrees(RobotContainer.isBlueAlliance() ? 180 : 0)
                )
            );
            System.out.println("Zeroed Swerve");
        }
    }

    public Command zeroGyroCommand() {

        return Commands.runOnce(() -> {resetOdometry(FieldHelpers.rotateBlueFieldCoordinates(new Translation2d(Meter.of(2), Meter.of(4)), RobotContainer.isRedAlliance()));});
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

    public ChassisSpeeds getRobotChassisSpeeds() {
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

    public Angle getAngle() {
        if (Constants.SwerveConstants.ENABLED == false) {
            return Degree.of(0);
        }

        return Degree.of(swerveDrive.getPose().getRotation().getDegrees());
    }

    public AngularVelocity getAngularVelocity() {
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

    public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {
        if (Constants.SwerveConstants.ENABLED == false) {
            return;
        }

        swerveDrive.addVisionMeasurement(robotPose, timestamp, stdDevs);
    }

    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        if (Constants.SwerveConstants.ENABLED == false) {
            return;
        }

        swerveDrive.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        if (Constants.SwerveConstants.ENABLED == false) {
            return;
        }

        swerveDrive.setChassisSpeeds(speeds); 
    }


    public Command driveFieldOriented(Supplier<ChassisSpeeds> speeds) {
        if (Constants.SwerveConstants.ENABLED == false) {
            return run(() -> {});
        }
        
        return run(() -> swerveDrive.driveFieldOriented(speeds.get()));
    }

    @Override
    public void periodic() {
        if (Constants.SwerveConstants.ENABLED == true) {
            RobotContainer.limelightSubsystem.getVisionEstimate();
        }
    }
}