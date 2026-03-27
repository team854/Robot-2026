package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.FieldHelpers;
import frc.robot.libraries.ProjectileSimulation.TargetErrorCode;
import frc.robot.libraries.ProjectileSimulation.TargetSolution;
import frc.robot.subsystems.turret.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class TurretAutoAimCommand extends Command {

    public TurretAutoAimCommand() {
        addRequirements(RobotContainer.turretSubsystem);
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.READY, 5);

        TargetSolution targetSolution = RobotContainer.calculationSubsystem.getTargetSolutions();
        if (targetSolution.errorCode() == TargetErrorCode.NONE) {


            Angle robotRelativeAngle = RobotContainer.turretSubsystem.getTurretPointAngle(targetSolution.launchYaw());
            RobotContainer.turretSubsystem.setTurretYaw(robotRelativeAngle);

            RobotContainer.turretSubsystem.setTurretPitch(Degree.of(90).minus(targetSolution.launchPitch()));

            AngularVelocity shooterSpeed = RobotContainer.calculationSubsystem.getProjectileSimulation().convertVelocityToShooterSpeed(targetSolution.launchSpeed(), Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, Constants.FuelPhysicsConstants.EFFICENCY);

            //RobotContainer.shooterSubsystem.setTargetSpeed(RotationsPerSecond.of(16.67 * 3));
            RobotContainer.shooterSubsystem.setTargetSpeed(shooterSpeed);
        }

        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.IDLE, 5);
    }
}
