package frc.robot;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
    public final class SwerveConstants {
        public static final boolean ENABLED = true;
        
        public static final File SWERVE_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve"); // File with swerve configs.
        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.25); // Maximum speed the swerve drive can go
        public static final Angle GYRO_OFFSET = Degree.of(0);

        public static final double DRIVE_P = 2;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;

        public static final double SWERVE_ACCELERATION_CONSTANT = 5.0;
    }

    public final class TurretConstants {
        public static final boolean ENABLED = true;

        public static final int TURRET_YAW_MOTOR_ID = 56;
        public static final boolean TURRET_YAW_MOTOR_INVERTED = false;
        public static final boolean TURRET_YAW_ENCODER_INVERTED = false;
        public static final boolean TURRET_YAW_ABSOLUTE_ENCODER_INVERTED = false;
        public static final double TURRET_YAW_GEAR_RATIO = 113.0 / 13.0; // Rotations of the motor for one rotation of the turret
        public static final double TURRET_YAW_COUNTS_PER_REV = 42;
        public static final double TURRET_YAW_FUSION_MULTIPLIER = 2.0;
        public static final double TURRET_YAW_P = 2.9;
        public static final double TURRET_YAW_I = 0.4;
        public static final double TURRET_YAW_D = 0.12;
        public static final Voltage TURRET_YAW_S = Volts.of(0.2);
        public static final double TURRET_YAW_V = 0.12; // Unit is V/(rad/s)
        public static final double TURRET_YAW_A = 0.018; // Unit is V/(rad/s^2)
        public static final AngularVelocity TURRET_YAW_MAX_VELOCITY = DegreesPerSecond.of(2000);
        public static final AngularAcceleration TURRET_YAW_MAX_ACCELERATION = DegreesPerSecondPerSecond.of(2400);
        public static final Angle TURRET_YAW_IZONE = Degree.of(5);
        public static final int TURRET_YAW_HOMING_SENSOR_DIO = 2;
        public static final Voltage TURRET_YAW_HOMING_SEARCHING_VOLTAGE = Volt.of(0.65);
        public static final Voltage TURRET_YAW_HOMING_REFINING_VOLTAGE = Volt.of(-0.25);
        public static final Angle TURRET_YAW_READY_THRESHOLD = Radian.of(0.1);
    
        public static final Angle TURRET_YAW_OFFSET = Degree.of(-1);
        public static final Angle TURRET_YAW_LOWER_LIMIT = Degree.of(-180);
        public static final Angle TURRET_YAW_UPPER_LIMIT = Degree.of(180);

        public static final int TURRET_PITCH_MOTOR_ID = 12;
        public static final boolean TURRET_PITCH_MOTOR_INVERTED = false;
        public static final double TURRET_PITCH_GEAR_RATIO = 160.0 / 14.0; // Rotations of the motor for one rotation of the pitch
        public static final double TURRET_PITCH_ZERO_OFFSET = 0;
        public static final double TURRET_PITCH_P = 4.5;
        public static final double TURRET_PITCH_I = 0;
        public static final double TURRET_PITCH_D = 0.02;
        public static final Voltage TURRET_PITCH_S = Volts.of(0.1);
        public static final Voltage TURRET_PITCH_G = Volts.of(0.145);
        public static final double TURRET_PITCH_V = 0.125; // Unit is V/(rad/s)
        public static final double TURRET_PITCH_A = 0.015; // Unit is V/(rad/s^2)
        public static final AngularVelocity TURRET_PITCH_MAX_VELOCITY = DegreesPerSecond.of(2000);
        public static final AngularAcceleration TURRET_PITCH_MAX_ACCELERATION = DegreesPerSecondPerSecond.of(1000);
        public static final Angle TURRET_PITCH_IZONE = Degree.of(5);
        public static final Angle TURRET_PITCH_READY_THRESHOLD = Radian.of(0.05);

        public static final Angle TURRET_PITCH_UPPER_LIMIT = Degree.of(40);
        public static final Angle TURRET_PITCH_LOWER_LIMIT = Degree.of(1);
        public static final Translation3d TURRET_PIVOT_OFFSET = new Translation3d(
            -0.1675,
            0.0975,
            0.5
        ); // In meters
        public static final Distance TURRET_PIVOT_FUEL_OFFSET = Meter.of(0.22);

        public static final Angle TURRET_STOWED_PITCH_ANGLE = TURRET_PITCH_LOWER_LIMIT;

        public static final Time TURRET_LATENCY = Second.of(0.05);
    }

    public final class ShooterConstants {
        public static final boolean ENABLED = true;

        public static final int SHOOTER_MOTOR_1_ID = 15;
        public static final boolean SHOOTER_MOTOR_1_INVERTED = true;

        public static final int SHOOTER_MOTOR_2_ID = 16;
        public static final boolean SHOOTER_MOTOR_2_INVERTED = true;

        public static final double SHOOTER_GEAR_RATIO = 1; // Rotations of the motor for one rotation of the wheels

        public static final double SHOOTER_P = 1.1;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.01;
        public static final Voltage SHOOTER_S = Volts.of(0.1);
        public static final double SHOOTER_V = 0.118; // Unit is V/(rotations/s)
        public static final double SHOOTER_A = 0.005; // Unit is V/(rotations/s^2)
        public static final Distance SHOOTER_WHEEL_RADIUS = Inch.of(1.93);
        public static final AngularVelocity SHOOTER_MAX_VELOCITY = RotationsPerSecond.of(91);
        public static final AngularVelocity SHOOTER_MIN_VELOCITY = RotationsPerSecond.of(5);
        public static final AngularVelocity SHOOTER_READY_THRESHOLD = RotationsPerSecond.of(8);

        public static final Angle SHOOTER_YAW_DEADZONE = Degree.of(10);
    }
    
    public final class LightConstants {
        public static final int LIGHT_PORT = 0;

        public static final int LIGHT_LENGTH = 198;

        public static final LEDPattern COLOR_SHOOTER_ON = LEDPattern.solid(new Color(0, 255, 0));
        public static final LEDPattern COLOR_SHOOTER_OFF = LEDPattern.solid(new Color(255, 0, 0));
        public static final LEDPattern COLOR_INVALID_SHOT = LEDPattern.solid(new Color(0, 0, 255));
        public static final LEDPattern COLOR_TURRET_HOMING = LEDPattern.solid(Color.kPurple);
        public static final LEDPattern COLOR_BOT_DISABLED = LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), Meter.of(1.0 / 144.0));
    }

    public final class KickerConstants {
        public static final boolean ENABLED = true;
        public static final int KICKER_MOTOR_ID = 28;
        public static final boolean KICKER_MOTOR_INVERTED = false;
        public static final Voltage KICKER_MOTOR_VOLTAGE = Volt.of(12);
        public static final Current KICKER_REVERSE_CURRENT = Amp.of(400);
    }

    public final class SpindexerConstants {
        public static final boolean ENABLED = true;
        public static final int SPINDEXER_MOTOR_ID = 29;
        public static final boolean SPINDEXER_MOTOR_INVERTED = true;
        public static final Voltage SPINDEXER_MOTOR_VOLTAGE = Volt.of(12);
        public static final Current SPINDEXER_REVERSE_CURRENT = Amp.of(400);
    }

    public final class OperatorConstants {
        public static final double DEADBAND = 0.1;
        public static final double SWERVE_TRANSLATION_SCALE = 1;
        public static final double SWERVE_ROTATION_SCALE = 1;
        public static final double SWERVE_EXPONENT = 2;
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public final class IntakeConstants {
        public static final boolean ENABLED = true;

        public static final int INTAKE_DEPLOYMENT_MOTOR_ID_1 = 13;
        public static final boolean INTAKE_DEPLOYMENT_MOTOR_INVERTED_1 = false;

        public static final int INTAKE_DEPLOYMENT_MOTOR_ID_2 = 17;
        public static final boolean INTAKE_DEPLOYMENT_MOTOR_INVERTED_2 = true;

        public static final Voltage INTAKE_DEPLOYMENT_MOTOR_VOLTAGE = Volt.of(2.0);
        public static final Voltage INTAKE_DEPLOYMENT_MOTOR_FORCE_VOLTAGE = Volt.of(8);
        public static final Voltage INTAKE_DEPLOYMENT_MOTOR_HOLDING_VOLTAGE = Volt.of(0.35);

        public static final int INTAKE_MOTOR_ID_1 = 14;
        public static final boolean INTAKE_MOTOR_INVERTED_1 = true;

        public static final int INTAKE_MOTOR_ID_2 = 11;
        public static final boolean INTAKE_MOTOR_INVERTED_2 = false;

        public static final Voltage INTAKE_MOTOR_VOLTAGE = Volt.of(9);

        public static final int INTAKE_RETRACTED_SENSOR_DIO = 0;
        public static final int INTAKE_DEPLOYED_SENSOR_DIO = 1;
    }

    public final class HealthConstants {
        public static final Time CYCLE_TIME = Second.of(1);

        public static final Time CAN_ERROR_PERSIST = Second.of(2.5);
        public static final Time LIMELIGHT_ERROR_PERSIST = Second.of(1);
        public static final Time ABSOLUTE_ENCODER_ERROR_PERSIST = Second.of(1);

        public static final Voltage LOW_BATTERY_THRESHOLD = Volt.of(11.9);
    }

    public final class FuelPhysicsConstants {
        public static final int TPS = 20;
        public static final int MAX_STEPS = 10;
        public static final double DRAG_CONSTANT = 0.6;
        public static final double EFFICENCY = 0.42;
        public static final double ROT_DRAG_CONSTANT = 0.05;
        public static final double LIFT_CONSTANT = 0.35;
        public static final double CROSS_SECTION_AREA = 0.01767;
        public static final Mass MASS = Pound.of(0.5);
        public static final double FLUID_DENSITY = 1.2754;
        public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.80665);
    }

    public final class FieldConstants {
        public static final Distance FIELD_SIZE_X = Meter.of(16.54);
        public static final Distance FIELD_SIZE_Y = Meter.of(8.07);
        public static final Distance HUB_SIDE_DISTANCE = Meter.of(4.62344);
        public static final Distance HUB_TARGET_HEIGHT = Meter.of(1.8);
        public static final Distance PASS_SIDE_DISTANCE = Meter.of(1);
        public static final Distance PASS_OFFSET = Meter.of(2);
        public static final Distance TRENCH_OFFSET = Meter.of(3);
        public static final Distance TRENCH_RADIUS = Meter.of(1);

        public static final Distance FIELD_CHECK_DISTANCE = Meter.of(1);
    }

    public final class LimelightConstants {
        public static final String[] LIMELIGHT_NAMES = new String[] {"limelight-b", "limelight-bl"};

        public static final double SINGLE_TAG_STARTING_STD_DEV = 0.9;
        public static final double SINGLE_TAG_DISTANCE_FACTOR = 0.5;

        public static final double MULTI_TAG_STARTING_STD_DEV = 0.25;
        public static final double MULTI_TAG_DISTANCE_FACTOR = 0.15;
    }

    public final class QuestConstants {
        public static final Transform3d ROBOT_TO_QUEST = new Transform3d(-0.2675, -0.2225, 0.52, new Rotation3d(0, 15, -90));

        public static final double INITAL_STD_DEV_THRESHOLD = 1.2;
        public static final double STD_DEV_THRESHOLD = 0.7;

        public static final Matrix<N3,
         N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.5,
            0.5,
            Double.MAX_VALUE
        );
    }
}
