package frc.robot.commands.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.KickerSubsystem.KickerState;

public class ActivateKickerCommand extends Command {
    
    public ActivateKickerCommand() {
        addRequirements(RobotContainer.kickerSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.kickerSubsystem.requestDesiredState(KickerState.READY, 5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.kickerSubsystem.requestDesiredState(KickerState.IDLE, 5);
    }
}
