package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.libraries.LimelightHelpers;
import frc.robot.libraries.PoseHelpers;
import swervelib.SwerveDrive;

public class LimelightSubsystem extends SubsystemBase {
    
    private LimelightHelpers.PoseEstimate limelightMeasurement;


    public LimelightSubsystem() {
        
    }

    @Override
    public void periodic() {
        
    }

    public void getVisionEstimate() {

        // LimelightHelpers.SetIMUMode("limelight", 4);

        double robotAngle = RobotContainer.swerveSubsystem.getAngle().in(Degree);

        LimelightHelpers.SetRobotOrientation("limelight", robotAngle, RobotContainer.swerveSubsystem.getAngularVelocity().in(DegreesPerSecond), 0.0, 0.0, 0.0, 0.0);

        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        try {
            if (limelightMeasurement != null && limelightMeasurement.pose != null) {
                if (limelightMeasurement.tagCount > 0) {
                    double distance = limelightMeasurement.avgTagDist;

                    double stdDevs = Double.MAX_VALUE;
                    if (limelightMeasurement.tagCount == 1) {
                        if (distance >= 5) {
                            return;
                        }

                        stdDevs = (1 + (Math.pow(distance, 2.0) * 0.5));
                    } else {
                        stdDevs = (0.4 + (distance * 0.1));
                    }

                    RobotContainer.swerveSubsystem.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds, VecBuilder.fill(stdDevs, stdDevs, 99999999));
                    SmartDashboard.putNumber("Limelight/StdDevs", stdDevs);
                    SmartDashboard.putBoolean("LimeLight/Tag", true);
                    SmartDashboard.putNumberArray("LimeLight/Position", PoseHelpers.convertPoseToNumbers(limelightMeasurement.pose));
                } else {
                    SmartDashboard.putBoolean("LimeLight/Tag", false);
                }
            }
        }
        catch (Exception e) {
            System.err.println("An error occurred: " + e.getMessage());
            e.printStackTrace();
        }
    }
}
