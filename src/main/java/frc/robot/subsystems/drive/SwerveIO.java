package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import swervelib.SwerveDrive;

public interface SwerveIO {
    default void resetOdometry(Pose2d initialPose) {}

    default Pose2d getPose2d() {return new Pose2d();}

    default SwerveDrive getSwerveDrive() {return null;}

    default ChassisSpeeds getRobotChassisSpeeds() {return new ChassisSpeeds();}
    default ChassisSpeeds getFieldChassisSpeeds() {return new ChassisSpeeds();}

    default void driveFieldOriented(ChassisSpeeds speeds) {}
    default void driveRobotRelative(ChassisSpeeds speeds) {}

    default double getAngle() {return 0;}

    default double getAngularVelocity() {return 0;}

    default void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {}

    default PIDConstants getHeadingPIDConstants() {return new PIDConstants(0, 0, 0);}

    default boolean checkCANError() {return false;}
    default boolean checkAngleAbsoluteEncodersError() {return false;}
}
