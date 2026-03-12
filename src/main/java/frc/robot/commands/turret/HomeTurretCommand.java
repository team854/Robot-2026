package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class HomeTurretCommand extends Command{

    private boolean hasStartedHoming = false;
    
    public HomeTurretCommand() {
        addRequirements(RobotContainer.turretSubsystem);
    }
    
    @Override
    public void initialize() {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.HOMING, 5);
        hasStartedHoming = false;
    }

    @Override
    public void execute() {
        if (RobotContainer.turretSubsystem.getCurrentState() == TurretState.HOMING) {
            hasStartedHoming = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (!hasStartedHoming) {
            return false;
        }

        return RobotContainer.turretSubsystem.getCurrentState() != TurretState.HOMING;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.IDLE, 5);
    }
}
