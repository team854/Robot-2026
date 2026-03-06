package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class ToggleManualCommand extends Command {
    
    private final DoubleSupplier turretYawSupplier;
    private final DoubleSupplier turretPitchSupplier;

    public ToggleManualCommand(DoubleSupplier turretYawSupplier, DoubleSupplier turretPitchSupplier) {
        this.turretYawSupplier = turretYawSupplier;
        this.turretPitchSupplier = turretPitchSupplier;

        addRequirements(RobotContainer.turretSubsystem, RobotContainer.shooterSubsystem, RobotContainer.kickerSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.turretSubsystem.setOverrideState(TurretState.MANUAL);
        RobotContainer.shooterSubsystem.setOverrideState(ShooterState.READY);
    }

    @Override
    public void execute() {
        RobotContainer.turretSubsystem.setOverrideVoltages(
            Volt.of(turretYawSupplier.getAsDouble() * 6),
            Volt.of(turretPitchSupplier.getAsDouble() * 6)
        );

        RobotContainer.shooterSubsystem.setTargetSpeed(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY);


    }

    @Override
    public boolean isFinished() {
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turretSubsystem.setOverrideState(null);
        RobotContainer.turretSubsystem.setOverrideVoltages(Volt.of(0), Volt.of(0));

        RobotContainer.shooterSubsystem.setOverrideState(null);
        RobotContainer.shooterSubsystem.setTargetSpeed(RotationsPerSecond.of(0));
    }
}
