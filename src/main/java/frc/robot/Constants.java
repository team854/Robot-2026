package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {
    public static class SwerveConstants {
        public static final File SWERVE_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve"); // File with swerve configs
        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(10); // Maximum speed the swerve drive can go
        public static final Angle GYRO_OFFSET = Degree.of(0);
    }

    public final class TurretConstants {
        public static final int TURRET_YAW_MOTOR_ID = 10;
        public static final boolean TURRET_YAW_MOTOR_INVERTED = false;
        public static final boolean TURRET_YAW_ENCODER_INVERTED = false;
        public static final double TURRET_YAW_GEAR_RATIO = 1; // Rotations of the motor for one rotation of the turret
        public static final double TURRET_YAW_P = 0;
        public static final double TURRET_YAW_I = 0;
        public static final double TURRET_YAW_D = 0;
        public static final Voltage TURRET_YAW_S = Volts.of(0);
        public static final Voltage TURRET_YAW_V = Volts.of(0); // Unit is V/(rad/s)
        public static final Voltage TURRET_YAW_A = Volts.of(0); // Unit is V/(rad/s^2)
        public static final AngularVelocity TURRET_YAW_MAX_VELOCITY = DegreesPerSecond.of(10);
        public static final AngularAcceleration TURRET_YAW_MAX_ACCELERATION = DegreesPerSecondPerSecond.of(10);

        public static final int TURRET_PITCH_MOTOR_ID = 11;
        public static final boolean TURRET_PITCH_MOTOR_INVERTED = false;
        public static final boolean TURRET_PITCH_ENCODER_INVERTED = false;
        public static final double TURRET_PITCH_ZERO_OFFSET = 0;
        public static final double TURRET_PITCH_P = 0;
        public static final double TURRET_PITCH_I = 0;
        public static final double TURRET_PITCH_D = 0;
        public static final Voltage TURRET_PITCH_S = Volts.of(0);
        public static final Voltage TURRET_PITCH_G = Volts.of(0);
        public static final Voltage TURRET_PITCH_V = Volts.of(0); // Unit is V/(rad/s)
        public static final Voltage TURRET_PITCH_A = Volts.of(0); // Unit is V/(rad/s^2)
        public static final AngularVelocity TURRET_PITCH_MAX_VELOCITY = DegreesPerSecond.of(10);
        public static final AngularAcceleration TURRET_PITCH_MAX_ACCELERATION = DegreesPerSecondPerSecond.of(10);
        
        

        public static final Angle TURRET_UPPER_LIMIT = Degree.of(90);
        public static final Angle TURRET_LOWER_LIMIT = Degree.of(45);
        public static final Angle TURRET_YAW_LOWER_LIMIT = Degree.of(-135);
        public static final Angle TURRET_YAW_UPPER_LIMIT = Degree.of(135);
        public static final Translation3d TURRET_PIVOT_OFFSET = new Translation3d(
            0,
            0,
            0.18
        ); // In meters
        public static final Distance TURRET_PIVOT_FUEL_OFFSET = Meter.of(0.22);
    }

    public final class ShooterConstants {
        public static final int SHOOTER_MOTOR_1_ID = 15;
        public static final boolean SHOOTER_MOTOR_1_INVERTED = false;

        public static final int SHOOTER_MOTOR_2_ID = 16;
        public static final boolean SHOOTER_MOTOR_2_INVERTED = false;

        public static final double SHOOTER_GEAR_RATIO = 1; // Rotations of the motor for one rotation of the wheels

        public static final double SHOOTER_P = 0;
        public static final double SHOOTER_I = 0;
        public static final double SHOOTER_D = 0;
        public static final Voltage SHOOTER_S = Volts.of(0);
        public static final Voltage SHOOTER_V = Volts.of(0); // Unit is V/(rad/s)
        public static final Voltage SHOOTER_A = Volts.of(0); // Unit is V/(rad/s^2)
        public static final Distance SHOOTER_WHEEL_RADIUS = Inch.of(2);
        public static final AngularVelocity SHOOTER_MAX_VELOCITY = RotationsPerSecond.of(90);
        public static final AngularVelocity SHOOTER_MIN_VELOCITY = RotationsPerSecond.of(10);
        public static final AngularAcceleration SHOOTER_MAX_ACCELERATION = RotationsPerSecondPerSecond.of(100);
        
    }


    public final class KickerConstants {
        public static final int KICKER_MOTOR_ID = 28;
        public static final boolean KICKER_MOTOR_INVERTED = false;
    }

    public final class OperatorConstants {
        public static final double DEADBAND = 0.1;
        public static final double SWERVE_TRANSLATION_SCALE = 1;
        public static final double SWERVE_ROTATION_SCALE = 1;
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public final class FuelPhysicsConstants {
        public static final int TPS = 60;
        public static final int MAX_STEPS = 15;
        public static final double DRAG_CONSTANT = 0.5;
        public static final double ROT_DRAG_CONSTANT = 0.05;
        public static final double LIFT_CONSTANT = 0.4;
        public static final double CROSS_SECTION_AREA = 0.01767;
        public static final Mass MASS = Pound.of(0.5);
        public static final double FLUID_DENSITY = 1.2754;
        public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.80665);
    }

    public final class FieldConstants {
        public static final Distance FIELD_SIZE_X = Meter.of(16.54);
        public static final Distance FIELD_SIZE_Y = Meter.of(8.07);
        public static final Distance HUB_SIDE_DISTANCE = Meter.of(4.62344);
        public static final Distance HUB_TARGET_HEIGHT = Meter.of(1.9);
    }
}
