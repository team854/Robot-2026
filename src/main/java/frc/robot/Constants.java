package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pound;

import java.io.File;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {
    public static class SwerveConstants {
        public static final File SWERVE_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve"); // File with swerve configs
        public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(10); // Maximum speed the swerve drive can go
        public static final Angle GYRO_OFFSET = Degree.of(0);
    }

    public final class TurretConstants {
        public static final Angle TURRET_UPPER_LIMIT = Degree.of(80);
        public static final Angle TURRET_LOWER_LIMIT = Degree.of(-15);
        public static final Translation3d TURRET_PIVOT_OFFSET = new Translation3d(
            0,
            0,
            0.18
        ); //In meters
        public static final Distance TURRET_PIVOT_FUEL_OFFSET = Meter.of(0.22);
    }

    public final class ShooterConstants {
    }

    public final class OperatorConstants {
        public static final double DEADBAND = 0.1;
        public static final double SWERVE_TRANSLATION_SCALE = 1;
        public static final double SWERVE_ROTATION_SCALE = 1;
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public final class FuelPhysicsConstants {
        public static final double DRAG_CONSTANT = 0.5;
        public static final double CROSS_SECTION_AREA = 0.01767;
        public static final Mass MASS = Pound.of(0.5);
        public static final double FLUID_DENSITY = 1.2754;
        public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.80665);
    }
}
