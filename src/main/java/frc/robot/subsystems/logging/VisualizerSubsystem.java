package frc.robot.subsystems.logging;

import static edu.wpi.first.units.Units.Degree;
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
import frc.robot.subsystems.intake.IntakeDeploymentSubsystem.IntakeDeploymentState;
import frc.robot.subsystems.turret.ShooterSubsystem.ShooterState;

public class VisualizerSubsystem extends SubsystemBase {
    private final boolean isSimulation = Robot.isSimulation();
    
    public VisualizerSubsystem() {

    }

    @Override
    public void periodic() {
        if (isSimulation) {
            Translation3d targetPosition = RobotContainer.calculationSubsystem.getTargetPosition();

            Pose2d robotPose = RobotContainer.swerveSubsystem.getPose2d();

            Translation3d robotTargetRelative = new Translation3d(
                targetPosition.getX() - robotPose.getX(),
                targetPosition.getY() - robotPose.getY(),
                targetPosition.getZ()
            );
            
            LinearVelocity launchSpeed = RobotContainer.calculationSubsystem.getProjectileSimulation().convertShooterSpeedToVelocity(
                RobotContainer.shooterSubsystem.getTargetSpeed(),
                Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS,
                Constants.FuelPhysicsConstants.EFFICENCY
            );

            Angle launchYaw = Radian.of(
                robotPose.getRotation().getRadians() - RobotContainer.turretSubsystem.getTurretTargetYaw().in(Radian) +  + Math.PI
            );

            ChassisSpeeds fieldSpeeds = RobotContainer.swerveSubsystem.getFieldChassisSpeeds();

            Double[] positionHistory = RobotContainer.calculationSubsystem.getProjectileSimulation().simulateLaunch(
                launchSpeed,
                Degree.of(90).minus(RobotContainer.turretSubsystem.getTurretTargetPitch()),
                launchYaw,
                RadiansPerSecond.of(-(launchSpeed.in(MetersPerSecond) / RobotContainer.calculationSubsystem.getProjectileSimulation().projectileRadius)),
                RadiansPerSecond.of(0), 
                new Translation2d(
                    fieldSpeeds.vxMetersPerSecond,
                    fieldSpeeds.vyMetersPerSecond
                ),
                robotTargetRelative,
                Radian.of(Math.atan2(robotTargetRelative.getY(), robotTargetRelative.getX())),
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
        SmartDashboard.putBoolean("Shooter/Active", RobotContainer.shooterSubsystem.getDesiredState() == ShooterState.READY);
        SmartDashboard.putBoolean("Intake Deployment/Deployed", RobotContainer.intakeDeploymentSubsystem.getDesiredState() == IntakeDeploymentState.DEPLOYED);
    }
}
