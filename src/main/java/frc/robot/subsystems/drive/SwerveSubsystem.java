package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{
    public RobotConfig config;

    private final SwerveDrive swerveDrive;

    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI); // Enables connection to the RIO

    private final Rotation3d gyroOffset = new Rotation3d(
        0.0,
        0.0,
        Constants.SwerveConstants.GYRO_OFFSET.in(Radian)
    );

    public SwerveSubsystem() {
    
        swervelib.telemetry.SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {

            // Will throw error if no swerve directory (Thats the purpose)
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
        
       
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e){
            e.printStackTrace();    
        }
        
        AutoBuilder.configure(
            this::getPose, 
            this::resetPose, 
            this::getRobotRelativeSpeeds, 
            (speeds, feedForwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController( 
                    new PIDConstants(Constants.SwerveConstants.Path_Finder_PID_P,Constants.SwerveConstants.Path_Finder_PID_I, Constants.SwerveConstants.Path_Finder_PID_D), 
                    new PIDConstants(Constants.SwerveConstants.Path_Finder_PID_P,Constants.SwerveConstants.Path_Finder_PID_I, Constants.SwerveConstants.Path_Finder_PID_D) 
            ),
            config,
            () -> {
            

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this 
    );

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
    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }
    
    public void driveRobotRelative(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }   
}
