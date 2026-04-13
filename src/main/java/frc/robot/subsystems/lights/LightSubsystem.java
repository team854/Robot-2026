package frc.robot.subsystems.lights;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.ProjectileSimulation.TargetErrorCode;
import frc.robot.libraries.SubsystemStateMachine;
import frc.robot.subsystems.logging.HealthSubsystem.ErrorCode;
import frc.robot.subsystems.turret.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class LightSubsystem extends SubsystemBase {

    private final AddressableLED ledStrip1 = new AddressableLED(Constants.LightConstants.LIGHT_PORT_1);
    private final AddressableLED ledStrip2 = new AddressableLED(Constants.LightConstants.LIGHT_PORT_2);

    private final AddressableLEDBuffer ledBuffer1 = new AddressableLEDBuffer(Constants.LightConstants.LIGHT_LENGTH_1);
    private final AddressableLEDBuffer ledBuffer2 = new AddressableLEDBuffer(Constants.LightConstants.LIGHT_LENGTH_2);
    
    public LightSubsystem() {

        ledStrip1.setLength(Constants.LightConstants.LIGHT_LENGTH_1);
        ledStrip1.start();
        
        ledStrip2.setLength(Constants.LightConstants.LIGHT_LENGTH_2);
        ledStrip2.start();
    }

    private LEDPattern determineLEDPattern() {

        ErrorCode errorCode = RobotContainer.healthSubsystem.getCurrentDisplayError();
        
        if (errorCode != null) {
            return errorCode.ledPattern();
        }

        if (DriverStation.isEnabled()) {
            if (RobotContainer.turretSubsystem.getCurrentState() == TurretState.HOMING) {
                return Constants.LightConstants.COLOR_TURRET_HOMING;
            }
            
            if (RobotContainer.calculationSubsystem.getTargetSolutions().errorCode() != TargetErrorCode.NONE) {
                if (Timer.getFPGATimestamp() % 0.4d > 0.2) {
                    return Constants.LightConstants.COLOR_INVALID_SHOT;
                }
            }

            if (RobotContainer.shooterSubsystem.getDesiredState() == ShooterState.READY) {
                return Constants.LightConstants.COLOR_SHOOTER_ON;
            } else {
                return Constants.LightConstants.COLOR_SHOOTER_OFF;
            }


        } else {
            return Constants.LightConstants.COLOR_BOT_DISABLED;
        }



    }

    @Override
    public void periodic() {
        LEDPattern ledPattern = determineLEDPattern();

        ledPattern.applyTo(ledBuffer1);
        ledStrip1.setData(ledBuffer1);

        ledPattern.applyTo(ledBuffer2);
        ledStrip2.setData(ledBuffer2);
    }
}
