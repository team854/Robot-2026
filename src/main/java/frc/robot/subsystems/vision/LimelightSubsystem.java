package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.libraries.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    
    private LimelightHelpers.PoseEstimate limelightMeasurement;

    public LimelightSubsystem() {

    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation("limelight", RobotContainer.swerveSubsystem.getNavXAngle().in(Degree), 0.0, 0.0, 0.0, 0.0, 0.0);
        
        getVisionEstimate();
    }

    public void getVisionEstimate() {
        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        try {
            if (limelightMeasurement != null && limelightMeasurement.pose != null) {
                if (limelightMeasurement.pose.getX() != 0) {
                    RobotContainer.swerveSubsystem.getSwerveDrive().addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
                    SmartDashboard.putBoolean("LimeLight/Tag", true);
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
