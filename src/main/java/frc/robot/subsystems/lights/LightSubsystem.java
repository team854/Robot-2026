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

public class LightSubsystem extends SubsystemBase {
    private final AddressableLED ledStrip = new AddressableLED(Constants.LightConstants.LIGHT_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LightConstants.LIGHT_LENGTH);
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(1));

    public LightSubsystem() {
        ledStrip.setLength(Constants.LightConstants.LIGHT_LENGTH);
        rainbow.applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    @Override
    public void periodic() {
        rainbow.applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
    }
}
