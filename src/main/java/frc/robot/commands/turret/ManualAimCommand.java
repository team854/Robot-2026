package frc.robot.commands.turret;

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
        SmartDashboard.putNumber("Turret/TargetYawAngle", 0);
        SmartDashboard.putNumber("Turret/TargetPitchAngle", 70);
    }

    @Override
    public void execute() {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.READY, 5);

        RobotContainer.turretSubsystem.setTurretYaw(Degree.of(SmartDashboard.getNumber("Turret/TargetYawAngle", 0)), false);
        RobotContainer.turretSubsystem.setTurretPitch(Degree.of(SmartDashboard.getNumber("Turret/TargetPitchAngle", 70)));
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.IDLE, 5);
    }
}
