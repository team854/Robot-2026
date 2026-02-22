package frc.robot.subsystems.logging;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.libraries.FieldHelpers;
import frc.robot.libraries.PoseHelpers;

public class VisualizerSubsystem extends SubsystemBase {
    private final boolean isSimulation = Robot.isSimulation();
    private Translation3d hubPosition;
    
    public VisualizerSubsystem() {
        hubPosition = FieldHelpers.rotateBlueFieldCoordinates(new Translation3d(
            Constants.FieldConstants.HUB_SIDE_DISTANCE.in(Meter),
            Constants.FieldConstants.FIELD_SIZE_Y.in(Meter) / 2.0,
            Constants.FieldConstants.HUB_TARGET_HEIGHT.in(Meter)
        ), !RobotContainer.isBlueAlliance());
    }

    @Override
    public void periodic() {
        if (isSimulation) {

            Pose2d robotPose = RobotContainer.swerveSubsystem.getPose2d();

            Translation3d robotHubRelative = new Translation3d(
                hubPosition.getX() - robotPose.getX(),
                hubPosition.getY() - robotPose.getY(),
                hubPosition.getZ()
            );
            
            LinearVelocity launchSpeed = RobotContainer.projectileSimulation.convertShooterSpeedToVelocity(
                RobotContainer.shooterSubsystem.getTargetSpeed(),
                Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS,
                0.5
            );

            Angle launchYaw = Radian.of(
                RobotContainer.turretSubsystem.getTurretTargetYaw().in(Radian) + robotPose.getRotation().getRadians()
            );

            ChassisSpeeds fieldSpeeds = RobotContainer.swerveSubsystem.getFieldChassisSpeeds();

            Double[] positionHistory = RobotContainer.projectileSimulation.simulateLaunch(
                launchSpeed,
                RobotContainer.turretSubsystem.getTurretTargetPitch(),
                launchYaw,
                RadiansPerSecond.of(-(launchSpeed.in(MetersPerSecond) / RobotContainer.projectileSimulation.projectileRadius)),
                RadiansPerSecond.of(0), 
                new Translation2d(
                    fieldSpeeds.vxMetersPerSecond,
                    fieldSpeeds.vyMetersPerSecond
                ),
                robotHubRelative,
                Radian.of(Math.atan2(robotHubRelative.getY(), robotHubRelative.getX())),
                true,
                Constants.FuelPhysicsConstants.TPS
            );

            Translation3d[] globalPositionHistory = new Translation3d[positionHistory.length / 3]; 
            for (int index = 0; index < positionHistory.length; index+=3) {
                globalPositionHistory[index / 3] = new Translation3d(
                    positionHistory[index] + robotPose.getX(),
                    positionHistory[index + 1] + robotPose.getY(),
                    positionHistory[index + 2]
                );
            }

            SmartDashboard.putNumberArray("Logging/Trajectory Visualizer", PoseHelpers.convertTranslationArrayToNumbers(globalPositionHistory));
        }
    }
}
