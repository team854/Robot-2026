package frc.robot.subsystems.lights;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.SubsystemStateMachine;

public class LightSubsystem extends SubsystemStateMachine<frc.robot.subsystems.lights.LightSubsystem.LightState> {

    public enum LightState {
        OFF,
        RED,
        GREEN
    }

    private final AddressableLED ledStrip = new AddressableLED(Constants.LightConstants.LIGHT_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LightConstants.LIGHT_LENGTH);
    
    public LightSubsystem() {
        super(LightState.OFF);

        ledStrip.setLength(Constants.LightConstants.LIGHT_LENGTH);
        ledStrip.start();
    }

    @Override
    public void periodic() {
        transitionTo(getDesiredState());

        ledStrip.setData(ledBuffer);
    }
}
