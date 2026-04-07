package frc.robot.subsystems.logging;

import static edu.wpi.first.units.Units.Second;

import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HealthSubsystem extends SubsystemBase {
    public record ErrorCode (
        int priority,
        String subsystemName,
        String errorText,
        LEDPattern ledPattern,
        boolean showEnabled,
        boolean showDisabled
    ) {};

    private final Set<ErrorCode> activeErrors;

    public HealthSubsystem() {
        activeErrors = new HashSet<>(); 
    }

    public void reportError(ErrorCode errorCode) {
        activeErrors.add(errorCode);
    }

    public void clearError(ErrorCode errorCode) {
        activeErrors.remove(errorCode);
    }

    public ErrorCode getCurrentDisplayError() {
        boolean isEnabled = DriverStation.isEnabled();

        List<ErrorCode> validErrors = activeErrors.stream()
                .filter(error -> isEnabled ? error.showEnabled() : error.showDisabled())
                .toList();

        if (validErrors.isEmpty()) {
            return null;
        }

        int highestPriority = validErrors.stream()
                .mapToInt(ErrorCode::priority)
                .max()
                .getAsInt();

        List<ErrorCode> tiedErrors = validErrors.stream()
                .filter(error -> error.priority() == highestPriority)
                .sorted(Comparator.comparing(ErrorCode::subsystemName).thenComparing(ErrorCode::errorText))
                .toList();

        if (tiedErrors.size() == 1) {
            return tiedErrors.get(0);
        } else {
            int index = (int) ((Timer.getFPGATimestamp() / Constants.HealthConstants.CYCLE_TIME.in(Second)) % tiedErrors.size());
            return tiedErrors.get(index);
        }
    }

}
