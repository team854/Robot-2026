package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ManualAimCommand extends Command {



    public ManualAimCommand() {
        addRequirements(RobotContainer.turretSubsystem);
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Turret/TargetAngle", 0);
    }

    @Override
    public void execute() {
        RobotContainer.turretSubsystem.setDesiredState(TurretState.READY);
        RobotContainer.turretSubsystem.setTurretYaw(Degree.of(SmartDashboard.getNumber("Turret/TargetAngle", 0)));
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.setDesiredState(TurretState.IDLE);
    }
}
