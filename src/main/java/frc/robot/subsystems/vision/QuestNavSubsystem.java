package frc.robot.subsystems.vision;

import java.util.OptionalInt;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ErrorConstants;
import frc.robot.RobotContainer;
import frc.robot.libraries.FieldHelpers;
import frc.robot.libraries.PoseHelpers;
import frc.robot.subsystems.vision.LimelightSubsystem.VisionRejection;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    public enum VisionRejection {
        NONE,
        OUT_OF_FIELD_BOUNDS,
        LOW_BATTERY
    }


    public record VisionStdDevs (
        Matrix<N3, N1> stdDevs,
        VisionRejection visionRejection
    ) {}

    private final QuestNav questNav;
    
    public QuestNavSubsystem() {
        questNav = new QuestNav();
    }

    public void resetPose(Pose3d resetPose) {
        questNav.setPose(resetPose.transformBy(Constants.QuestConstants.ROBOT_TO_QUEST));
    }

    public VisionStdDevs calculateVisionStdDevs(Pose3d visionPose, OptionalInt battery) {
        if (!FieldHelpers.poseInField(visionPose)) {
            return new VisionStdDevs(VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE), VisionRejection.OUT_OF_FIELD_BOUNDS);
        }

        if(battery.isPresent()) {
            if (battery.getAsInt() < 20) {
                return new VisionStdDevs(VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE), VisionRejection.LOW_BATTERY);
            }
        }

        return new VisionStdDevs(Constants.QuestConstants.QUESTNAV_STD_DEVS, VisionRejection.NONE);
    }


    public void getVisionEstimate() {
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {
                Pose3d questPose = questFrame.questPose3d();
                double timestamp = questFrame.dataTimestamp();

                Pose3d robotPose = questPose.transformBy(Constants.QuestConstants.ROBOT_TO_QUEST.inverse());

                VisionStdDevs stdDevs = calculateVisionStdDevs(robotPose, questNav.getBatteryPercent());

                if (!stdDevs.stdDevs().equals(VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE))) {
                    RobotContainer.swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp, stdDevs.stdDevs());

                    SmartDashboard.putNumberArray("QuestNav/Position", PoseHelpers.convertPoseToNumbers(robotPose));
                }

                SmartDashboard.putString("QuestNav/StdDevs", stdDevs.stdDevs().toString());
                SmartDashboard.putString("QuestNav/Rejction", stdDevs.visionRejection().toString());
                
            }
        }
    }

    public void checkQuestHealth() {
        if (!questNav.isConnected()) {
            RobotContainer.healthSubsystem.reportError(getSubsystem(), ErrorConstants.QUEST_DISCONNECTED);
        } else {
            RobotContainer.healthSubsystem.clearError(getSubsystem(), ErrorConstants.QUEST_DISCONNECTED);
        }
    }
 
    @Override
    public void periodic() {
        questNav.commandPeriodic();

        SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
        SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
        SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());
        questNav.getBatteryPercent().ifPresent(
            battery -> SmartDashboard.putNumber("QuestNav/Battery Percent", battery));
        questNav.getTrackingLostCounter().ifPresent(
            count -> SmartDashboard.putNumber("QuestNav/Tracking Lost Count", count));

        checkQuestHealth();
    }
}
