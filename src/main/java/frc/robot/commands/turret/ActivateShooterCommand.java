package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ActivateShooterCommand extends Command{

    public ActivateShooterCommand() {
        addRequirements(RobotContainer.shooterSubsystem);
    }
    
    @Override
    public void initialize() {
        RobotContainer.shooterSubsystem.setTargetSpeed(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY);
    }

    @Override
    public void execute() {
        RobotContainer.shooterSubsystem.requestDesiredState(ShooterState.READY, 5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooterSubsystem.requestDesiredState(ShooterState.IDLE, 5);
    }
}
