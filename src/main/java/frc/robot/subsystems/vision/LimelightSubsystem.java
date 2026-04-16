package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.core.Mat;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ErrorConstants;
import frc.robot.RobotContainer;
import frc.robot.libraries.FieldHelpers;
import frc.robot.libraries.LimelightHelpers;
import frc.robot.libraries.PoseHelpers;
import swervelib.SwerveDrive;

public class LimelightSubsystem extends SubsystemBase {
    
    public enum VisionRejection {
        NONE,
        NO_TAGS,
        MEASUREMENT_NULL,
        SINGLE_TAG_MAX_DISTANCE,
        MAX_ANGULAR_VELOCITY,
        OUT_OF_FIELD_BOUNDS
    }

    public record VisionStdDevs (
        double stdDevs,
        VisionRejection visionRejection
    ) {}

    public record VisionMeasurement (
        Pose2d robotPose,
        Time timestamp,
        VisionStdDevs visionStdDevs
    ) {}

    private final Map<String, Double> lastHeartbeats = new HashMap<>();
    private final Map<String, Double> lastHeartbeatTimes = new HashMap<>();

    public LimelightSubsystem() {
        for (String limelight : Constants.LimelightConstants.LIMELIGHT_NAMES) {
            lastHeartbeats.put(limelight, 0.0);
            lastHeartbeatTimes.put(limelight, 0.0);
        }
    }

    public void checkLimelightHealth() {
        boolean limelightDisconnected = false;

        for (String limelight : Constants.LimelightConstants.LIMELIGHT_NAMES) {
            double heartbeat = LimelightHelpers.getLimelightNTDouble(limelight, "hb");
            double timestamp = Timer.getFPGATimestamp();
            if (heartbeat != lastHeartbeats.get(limelight)) {
                lastHeartbeats.put(limelight, heartbeat);
                lastHeartbeatTimes.put(limelight, timestamp);
            } else {
                if ((timestamp - lastHeartbeatTimes.get(limelight)) > Constants.HealthConstants.LIMELIGHT_ERROR_PERSIST.in(Second)) {
                    limelightDisconnected = true;
                }
            }
        }

        
        if (limelightDisconnected) {
            RobotContainer.healthSubsystem.reportError(getSubsystem(), ErrorConstants.LIMELIGHT_DISCONNECTED);
        } else {
            RobotContainer.healthSubsystem.clearError(getSubsystem(), ErrorConstants.LIMELIGHT_DISCONNECTED);
        }
    }

    @Override
    public void periodic() {
        checkLimelightHealth();
    }

    public VisionStdDevs calculateVisionStdDevs(Pose2d visionPose, int tagCount, double averageTagDistance, ChassisSpeeds robotChassisSpeeds, AngularVelocity robotAngularVelocity) {
        if (tagCount == 0) {
            return new VisionStdDevs(Double.MAX_VALUE, VisionRejection.NO_TAGS);
        }

        if (Math.abs(robotAngularVelocity.in(RadiansPerSecond)) > (Math.PI * 2)) {
            return new VisionStdDevs(Double.MAX_VALUE, VisionRejection.MAX_ANGULAR_VELOCITY);
        }

        if (!FieldHelpers.poseInField(visionPose)) {
            return new VisionStdDevs(Double.MAX_VALUE, VisionRejection.OUT_OF_FIELD_BOUNDS);
        }
        
        double robotTranslationalVelocity = Math.hypot(robotChassisSpeeds.vxMetersPerSecond, robotChassisSpeeds.vyMetersPerSecond);

        double stdDevs = Math.abs(robotAngularVelocity.in(DegreesPerSecond)) / 720.0;

        stdDevs += robotTranslationalVelocity / 10.0;

        if (tagCount == 1) {
            if (averageTagDistance >= 5) {
                return new VisionStdDevs(Double.MAX_VALUE, VisionRejection.SINGLE_TAG_MAX_DISTANCE);
            }

            stdDevs += (Constants.LimelightConstants.SINGLE_TAG_STARTING_STD_DEV + (Math.pow(averageTagDistance, 2.0) * Constants.LimelightConstants.SINGLE_TAG_DISTANCE_FACTOR));
        } else {
            stdDevs += (Constants.LimelightConstants.MULTI_TAG_STARTING_STD_DEV + (averageTagDistance * Constants.LimelightConstants.MULTI_TAG_DISTANCE_FACTOR));
        }

        stdDevs = Math.max(stdDevs, 0.05);

        return new VisionStdDevs(stdDevs, VisionRejection.NONE);
    }

    public List<VisionMeasurement> getVisionEstimate() {

        // LimelightHelpers.SetIMUMode("limelight", 4);

        Angle robotAngle = RobotContainer.swerveSubsystem.getAngle();
        AngularVelocity robotAngularVelocity = RobotContainer.swerveSubsystem.getAngularVelocity();
        ChassisSpeeds robotChassisSpeeds = RobotContainer.swerveSubsystem.getRobotChassisSpeeds();

        List<VisionMeasurement> visionMeasurements = new ArrayList<>();

        for (String limelight : Constants.LimelightConstants.LIMELIGHT_NAMES) {
            LimelightHelpers.SetRobotOrientation(limelight, robotAngle.in(Degree), robotAngularVelocity.in(DegreesPerSecond), 0.0, 0.0, 0.0, 0.0);
            
            try {
                LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

                VisionStdDevs stdDevs;
                if (limelightMeasurement != null && limelightMeasurement.pose != null) {
                    stdDevs = calculateVisionStdDevs(
                        limelightMeasurement.pose,
                        limelightMeasurement.tagCount,
                        limelightMeasurement.avgTagDist,
                        robotChassisSpeeds,
                        robotAngularVelocity
                    );

                } else {
                    stdDevs = new VisionStdDevs(Double.MAX_VALUE, VisionRejection.MEASUREMENT_NULL);
                }

                if (stdDevs.stdDevs() != Double.MAX_VALUE) {
                    RobotContainer.swerveSubsystem.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds, VecBuilder.fill(stdDevs.stdDevs(), stdDevs.stdDevs(), Double.MAX_VALUE));

                    visionMeasurements.add(new VisionMeasurement(limelightMeasurement.pose, Seconds.of(limelightMeasurement.timestampSeconds), stdDevs));
                    
                    SmartDashboard.putNumberArray("Limelight/" + limelight + "/Position", PoseHelpers.convertPoseToNumbers(limelightMeasurement.pose));
                    SmartDashboard.putNumber("Limelight/" + limelight + "/Tag Count", limelightMeasurement.tagCount);
                }

                SmartDashboard.putNumber("Limelight/" + limelight + "/StdDevs", stdDevs.stdDevs());
                SmartDashboard.putString("Limelight/" + limelight + "/Rejction", stdDevs.visionRejection().toString());
                SmartDashboard.putBoolean("Limelight/" + limelight + "/Tag", stdDevs.stdDevs() != Double.MAX_VALUE);

            } catch (Exception e) {
                System.err.println("An error occurred: " + e.getMessage());
                e.printStackTrace();
            }
        }

        return visionMeasurements;
    }
}
