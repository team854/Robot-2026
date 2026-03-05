package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeDeploymentSubsystem.IntakeDeploymentState;

public class DeployIntakeCommand extends Command{

    public DeployIntakeCommand() {
        addRequirements(RobotContainer.intakeDeploymentSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.intakeDeploymentSubsystem.requestDesiredState(IntakeDeploymentState.DEPLOYED, 5);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
