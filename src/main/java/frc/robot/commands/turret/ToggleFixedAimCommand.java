package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.ProjectileSimulation.TargetErrorCode;
import frc.robot.libraries.ProjectileSimulation.TargetSolution;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ToggleFixedAimCommand extends Command{
    
    public ToggleFixedAimCommand() {
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
            RobotContainer.turretSubsystem.setTurretYaw(Degree.of(0), false);

            RobotContainer.turretSubsystem.setTurretPitch(Degree.of(90).minus(targetSolution.launchPitch()));

            AngularVelocity shooterSpeed = RobotContainer.calculationSubsystem.getProjectileSimulation().convertVelocityToShooterSpeed(targetSolution.launchSpeed(), Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, Constants.FuelPhysicsConstants.EFFICENCY);

            //RobotContainer.shooterSubsystem.setTargetSpeed(RotationsPerSecond.of(9));
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
