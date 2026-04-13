package frc.robot.subsystems.logging;

import static edu.wpi.first.units.Units.Second;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HealthSubsystem extends SubsystemBase {

    /**
     * ErrorCode
     * 
     * @param priority An int representing the priority of the error. Priorities should generally follow this format:
     * <ul>
     *      <li>0-5: Warnings</li>
     *      <li>6-10: Errors</li>
     *      <li>11-15: Critical</li>
     * </ul>
     * @param errorText Text describing the error
     * @param ledPattern An {@link LEDPattern} that should be shown on the leds if this error is reported
     * @param showEnabled Should the pattern be shown when the robot is enabled
     * @param showDisabled Should the pattern be shown when the robot is disabled
    **/    
    public record ErrorCode (
        int priority,
        String errorText,
        LEDPattern ledPattern,
        boolean showEnabled,
        boolean showDisabled
    ) {};

    private final Map<ErrorCode, TreeSet<String>> activeErrors = new HashMap<>();

    public HealthSubsystem() {
        
    }

    public void reportError(String subsystem, ErrorCode errorCode) {
        if (activeErrors.get(errorCode) == null) {
            activeErrors.put(errorCode, new TreeSet<>());
        }

        activeErrors.get(errorCode).add(subsystem);
    }

    public void clearError(String subsystem, ErrorCode errorCode) {
        if (activeErrors.containsKey(errorCode)) {
            activeErrors.get(errorCode).remove(subsystem);
            
            if (activeErrors.get(errorCode).isEmpty()) {
                activeErrors.remove(errorCode);
            }
        }
    }

    public TreeSet<String> getErrorSubsystems(ErrorCode errorCode) {
        return activeErrors.getOrDefault(errorCode, new TreeSet<>());
    }

    public ErrorCode getCurrentDisplayError() {
        boolean isEnabled = DriverStation.isEnabled();

        List<ErrorCode> validErrors = activeErrors.keySet().stream()
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
                .sorted(Comparator.comparing(ErrorCode::errorText))
                .toList();

        if (tiedErrors.size() == 1) {
            return tiedErrors.get(0);
        } else {
            int index = (int) ((Timer.getFPGATimestamp() / Constants.HealthConstants.CYCLE_TIME.in(Second)) % tiedErrors.size());
            return tiedErrors.get(index);
        }
    }

    @Override
    public void periodic() {
        ErrorCode displayErrorCode = getCurrentDisplayError();

        String errorCodeText = "NONE";
        TreeSet<String> errorCodeSubsystems = new TreeSet<>();
        if (displayErrorCode != null) {
            errorCodeText = displayErrorCode.errorText;
            errorCodeSubsystems = getErrorSubsystems(displayErrorCode);
        }

        SmartDashboard.putString("Health/Display Error Text", errorCodeText);
        SmartDashboard.putString("Health/Display Error Subsystems", errorCodeSubsystems.toString());
    }

}
