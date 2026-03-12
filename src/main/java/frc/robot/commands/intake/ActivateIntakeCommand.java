package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;

public class ActivateIntakeCommand extends Command {
    public ActivateIntakeCommand() {
        addRequirements(RobotContainer.intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.intakeSubsystem.requestDesiredState(IntakeState.READY, 5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.intakeSubsystem.requestDesiredState(IntakeState.IDLE, 5);
    }
}
