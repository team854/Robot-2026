package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ManualStowTurretCommand extends Command {
    
    public ManualStowTurretCommand() {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.STOWED, 30);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.IDLE, 0);
    }
}
