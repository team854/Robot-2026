package frc.robot.libraries;

import static edu.wpi.first.units.Units.Meter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public final class PoseHelpers {
    public static double[] convertPoseToNumbers(Pose3d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            pose.getRotation().getQuaternion().getW(),
            pose.getRotation().getQuaternion().getX(),
            pose.getRotation().getQuaternion().getY(),
            pose.getRotation().getQuaternion().getZ()
        };
    }

    public static double[] convertPoseToNumbers(Pose2d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }

    public static double[] convertTranslationToNumbers(Translation3d translation) {
        return new double[] {
            translation.getX(),
            translation.getY(),
            translation.getZ()
        };
    }

    public static Double[] convertPoseArrayToNumbers(Pose3d[] poseArray) {
        List<Double> outputArray = new ArrayList<>();
		for (Pose3d pose : poseArray) {
			double[] poseDoubleList = PoseHelpers.convertPoseToNumbers(pose);
			for (double num : poseDoubleList) {
				outputArray.add(num);
			}
		}
		return outputArray.toArray(new Double[0]);
    }

    public static Double[] convertPoseArrayToNumbers(Pose2d[] poseArray) {
        List<Double> outputArray = new ArrayList<>();
		for (Pose2d pose : poseArray) {
			double[] poseDoubleList = PoseHelpers.convertPoseToNumbers(pose);
			for (double num : poseDoubleList) {
				outputArray.add(num);
			}
		}
		return outputArray.toArray(new Double[0]);
    }

    public static Double[] convertTranslationArrayToNumbers(Translation3d[] translationArray) {
        List<Double> outputArray = new ArrayList<>();
		for (Translation3d pose : translationArray) {
			double[] translationDoubleList = PoseHelpers.convertTranslationToNumbers(pose);
			for (double num : translationDoubleList) {
				outputArray.add(num);
			}
		}
		return outputArray.toArray(new Double[0]);
    }

    public static Pose3d convertTransformToPose(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    public static Distance calculatePoseDistance(Pose3d pose1, Pose3d pose2) {
        return Meter.of(pose1.getTranslation().getDistance(pose2.getTranslation()));
    }
}
