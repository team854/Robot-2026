package frc.robot.commands.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.spindexer.SpindexerSubsystem.SpindexerState;

public class ReverseSpindexerCommand extends Command{

    public ReverseSpindexerCommand() {
        addRequirements(RobotContainer.spindexerSubsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        RobotContainer.spindexerSubsystem.requestDesiredState(SpindexerState.READY_REVERSE, 5);
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