package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.logging.HealthSubsystem.ErrorCode;

public final class ErrorConstants {
    public static final double SPLIT_PERCENT = 0.475;

    public static final ErrorCode JOYSTICKS_DISCONNECTED = new ErrorCode(
        10,
        "No joysticks connected",
        LEDPattern.steps(Map.of(0, Color.kPurple, SPLIT_PERCENT, Color.kRed)),
        true,
        true
    );

    public static final ErrorCode DS_DISCONNECTED = new ErrorCode(
        10,
        "No driver station connection",
        LEDPattern.solid(new Color(255, 0, 0)).breathe(Second.of(1.0)),
        true,
        true
    );

    public static final ErrorCode MOTOR_CAN_ERROR = new ErrorCode(
        10,
        "A sparkmax has a CAN error",
        LEDPattern.steps(Map.of(0, Color.kGreen, SPLIT_PERCENT, Color.kYellow)),
        false,
        true
    );

    public static final ErrorCode SWERVE_ABSOLUTE_ENCODER_ERROR = new ErrorCode(
        10,
        "A swerve drive absolute encoder has an error",
        LEDPattern.steps(Map.of(0, Color.kWhite, SPLIT_PERCENT, Color.kPurple)),
        false,
        true
    );

    public static final ErrorCode LIMELIGHT_DISCONNECTED = new ErrorCode(
        10,
        "A limelight is disconnected",
        LEDPattern.steps(Map.of(0, Color.kGreen, SPLIT_PERCENT, Color.kBlue)),
        false,
        true
    );

    public static final ErrorCode QUEST_DISCONNECTED = new ErrorCode(
        10,
        "The quest is disconnected",
        LEDPattern.steps(Map.of(0, Color.kWhite, SPLIT_PERCENT, Color.kBlue)),
        false,
        true
    );

    public static final ErrorCode LOW_BATTERY_VOLTAGE = new ErrorCode(
        10,
        "The battery is below its low voltage threshold",
        LEDPattern.steps(Map.of(0, Color.kWhite, SPLIT_PERCENT, Color.kGreen)),
        false,
        true
    );
}
