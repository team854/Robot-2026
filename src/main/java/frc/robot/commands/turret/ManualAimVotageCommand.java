package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ManualAimVotageCommand extends Command {
    


    public ManualAimVotageCommand() {
        addRequirements(RobotContainer.turretSubsystem);
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Turret/TargetYawVoltage", 0);
    }

    @Override
    public void execute() {
        //System.out.println(SmartDashboard.getNumber("Turret/TargetYawAngle", 0));
        RobotContainer.turretSubsystem.setOverrideState(TurretState.MANUAL);
        RobotContainer.turretSubsystem.setOverrideVoltages(Volt.of(SmartDashboard.getNumber("Turret/TargetYawVoltage", 0)),Volt.of(0));
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.setOverrideState(null);
    }
}
