package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ResetTurretPitchCommand extends Command {

    private final Timer timer = new Timer();

    public ResetTurretPitchCommand() {
        addRequirements(RobotContainer.turretSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.IDLE, 20);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            RobotContainer.turretSubsystem.resetPitchPosition();
        }
    }
}
