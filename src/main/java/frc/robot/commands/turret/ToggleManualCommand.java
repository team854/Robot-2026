package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ToggleManualCommand extends Command {
    
    private final DoubleSupplier turretYawSupplier;
    private final DoubleSupplier turretPitchSupplier;

    private double targetYaw = 0; 
    private double targetPitch = 0; 

    public ToggleManualCommand(DoubleSupplier turretYawSupplier, DoubleSupplier turretPitchSupplier) {
        this.turretYawSupplier = turretYawSupplier;
        this.turretPitchSupplier = turretPitchSupplier;
        

        addRequirements(RobotContainer.turretSubsystem);
    }

    @Override
    public void initialize() {
        

        targetYaw = RobotContainer.turretSubsystem.getTurretYaw().in(Degree);
        targetPitch = RobotContainer.turretSubsystem.getTurretPitch().in(Degree);
    }

    @Override
    public void execute() {

        RobotContainer.turretSubsystem.requestDesiredState(TurretState.MANUAL, 15);
        
        targetYaw = turretYawSupplier.getAsDouble() * 0.1;
        targetPitch += turretPitchSupplier.getAsDouble() * 0.1;

        targetYaw = MathUtil.clamp(targetYaw, Constants.TurretConstants.TURRET_YAW_LOWER_LIMIT.in(Degree), Constants.TurretConstants.TURRET_YAW_UPPER_LIMIT.in(Degree));
        targetPitch = MathUtil.clamp(targetPitch, Constants.TurretConstants.TURRET_PITCH_LOWER_LIMIT.in(Degree), Constants.TurretConstants.TURRET_PITCH_UPPER_LIMIT.in(Degree));
        
        

        RobotContainer.turretSubsystem.setOverrideAngles(
            Degree.of(targetYaw),
            Degree.of(targetPitch)
        );

        RobotContainer.shooterSubsystem.setTargetSpeed(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY);


    }

    @Override
    public boolean isFinished() {
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.requestDesiredState(TurretState.IDLE, 0);
        RobotContainer.shooterSubsystem.setTargetSpeed(RotationsPerSecond.of(0));
    }
}
