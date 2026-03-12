package frc.robot.commands.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.spindexer.SpindexerSubsystem.SpindexerState;
import frc.robot.subsystems.turret.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ActivateSpindexerCommand extends Command{

    public ActivateSpindexerCommand() {
        addRequirements(RobotContainer.spindexerSubsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.READY, 5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.READY, 5);
    }
}
