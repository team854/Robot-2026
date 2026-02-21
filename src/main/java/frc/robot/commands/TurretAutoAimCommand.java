package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.FieldHelpers;
import frc.robot.libraries.ProjectileSimulation.TargetErrorCode;
import frc.robot.libraries.ProjectileSimulation.TargetSolution;

public class TurretAutoAimCommand extends Command {
    private Translation3d hubPosition;

    public TurretAutoAimCommand() {
        addRequirements(RobotContainer.turretSubsystem, RobotContainer.shooterSubsystem);
    }
    
    @Override
    public void initialize() {
        hubPosition = FieldHelpers.rotateBlueFieldCoordinates(new Translation3d(
            Constants.FieldConstants.HUB_SIDE_DISTANCE.in(Meter),
            Constants.FieldConstants.FIELD_SIZE_Y.in(Meter) / 2.0,
            Constants.FieldConstants.HUB_TARGET_HEIGHT.in(Meter)
        ), !RobotContainer.isBlueAlliance());
    }

    @Override
    public void execute() {
        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose2d();

        Translation3d robotHubRelative = new Translation3d(
            hubPosition.getX() - robotPose.getX(),
            hubPosition.getY() - robotPose.getY(),
            hubPosition.getZ()
        );

        TargetSolution solution = RobotContainer.projectileSimulation.calculateLaunchAngleSimulation(
            RobotContainer.projectileSimulation.convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5),
            DegreesPerSecond.of(0),
            new Translation2d(0, 0),
            robotHubRelative,
            Constants.FuelPhysicsConstants.MAX_STEPS,
            Constants.FuelPhysicsConstants.TPS
            
        );

        SmartDashboard.putString("Auto Aim/Error Code", solution.errorCode().name());
        SmartDashboard.putString("Auto Aim/Solution Debug", solution.targetDebug().toString());
        SmartDashboard.putBoolean("Auto Aim/Error", solution.errorCode() == TargetErrorCode.NONE);
        if (solution.errorCode() == TargetErrorCode.NONE) {
            Angle robotRelativeAngle = RobotContainer.turretSubsystem.getTurretPointAngle(solution.launchYaw());
            RobotContainer.turretSubsystem.setTurretYaw(robotRelativeAngle);

            RobotContainer.turretSubsystem.setTurretPitch(solution.launchPitch());
        }

        System.out.println(solution);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
