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
import swervelib.SwerveDrive;

public class LimelightSubsystem extends SubsystemBase {
    
    private LimelightHelpers.PoseEstimate limelightMeasurement;

    private static final Matrix<N3, N1> visionStandardDevs = VecBuilder.fill(.7, .7, 99999999);

    public LimelightSubsystem() {
        
    }

    @Override
    public void periodic() {
        
    }

    public void getVisionEstimate() {

        LimelightHelpers.SetIMUMode("limelight", 4);

        double robotAngle = RobotContainer.swerveSubsystem.getAngle().in(Degree);

        LimelightHelpers.SetRobotOrientation("limelight", robotAngle, RobotContainer.swerveSubsystem.getAngularVelocity().in(DegreesPerSecond), 0.0, 0.0, 0.0, 0.0);

        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        try {
            if (limelightMeasurement != null && limelightMeasurement.pose != null) {
                if (limelightMeasurement.tagCount > 0) {
                    RobotContainer.swerveSubsystem.setVisionMeasurementStdDevs(visionStandardDevs);
                    RobotContainer.swerveSubsystem.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
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
