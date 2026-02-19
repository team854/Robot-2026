package frc.robot.subsystems.projectile;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

public class ProjectileSubsystem {
    private final double dragCoefficient = Constants.FuelPhysicsConstants.DRAG_CONSTANT;
    private final double rotDragCoefficient = Constants.FuelPhysicsConstants.ROT_DRAG_CONSTANT; 
    private final double crossSectionArea = Constants.FuelPhysicsConstants.CROSS_SECTION_AREA;
    private final double mass = Constants.FuelPhysicsConstants.MASS.in(Kilogram);
    private final double fluidDensity = Constants.FuelPhysicsConstants.FLUID_DENSITY;
    private final double gravity = Constants.FuelPhysicsConstants.GRAVITY.in(MetersPerSecondPerSecond);
    private final double projectileRadius = Math.sqrt(crossSectionArea / Math.PI);
    private final double momentOfInertia = 0.4 * mass * Math.pow(projectileRadius, 2);
    private final double liftCoefficient = Constants.FuelPhysicsConstants.LIFT_CONSTANT;


    public ProjectileSubsystem() {

    }

    /**
     * TargetErrorCode
     * <ul>
     *       <li>NONE = No error</li>
     *       <li>IDEAL_PITCH = Ideal pitch cannot reach</li>
     *       <li>EXCESSIVE_YAW = Yaw exceeds 540 degrees (3pi)</li>
     *       <li>PITCH_UPPER_LIMIT = Pitch exceeds arm upper limit</li>
     *       <li>PITCH_LOWER_LIMIT = Pitch exceeds arm lower limit</li>
     *       <li>HEIGHT_ERROR_HIGH = Height error exceeds 0.1 meters</li>
     *       <li>YAW_ERROR_HIGH = Yaw error exceeds 0.1 radians</li>
     *       <li>SPEED_UPPER_LIMIT = Speed exceeds speed upper limit</li>
     *       <li>SPEED_LOWER_LIMIT = Speed exceeds speed lower limit</li>
     *  </ul>
     */
    public enum TargetErrorCode {
        NONE,
        IDEAL_PITCH,
        EXCESSIVE_YAW,
        PITCH_UPPER_LIMIT,
        PITCH_LOWER_LIMIT,
        HEIGHT_ERROR_HIGH,
        YAW_ERROR_HIGH,
        SPEED_UPPER_LIMIT,
        SPEED_LOWER_LIMIT
    }

    public record TargetDebug (
        int pitchYawSteps,
        int speedSteps,
        double heightError,
        double yawError
    ) {};

    /**
     * TargetSolution
     * 
     * @param errorCode Stores if the solver has encountered an error
     * @param launchSpeed Stores the launch speed of the projectile as an {@link LinearVelocity}
     * @param launchPitch Stores the launch pitch of the projectile as an {@link Angle}
     * @param launchYaw Stores the launch yaw of the projectile as an {@link Angle}
     */
    public record TargetSolution (
        TargetErrorCode errorCode,
        LinearVelocity launchSpeed,
        Angle launchPitch,
        Angle launchYaw,
        TargetDebug targetDebug
    ) {};

    /**
     * Calculates the launch pitch needed to reach a target not accounting for drag.
     * 
     * @param launchSpeed The launch speed of the projectile as a {@link LinearVelocity}
     * @param horizontalDistance The targets horizontal distance from the robot as a {@link Distance}
     * @param heightOffset The height offset of the target from the robot as a {@link Distance}
     * @return The {@link Angle} that the projectile should be launched. Null if the projectile cannot reach the target
     */
    private Angle calculateLaunchPitchIdeal(LinearVelocity launchSpeed, Distance horizontalDistance, Distance heightOffset) {
        double launchSpeedMPS = launchSpeed.in(MetersPerSecond);

        if (Math.abs(horizontalDistance.in(Meter)) < 1e-5) {
            return null; 
        }

        double discriminant = Math.pow(launchSpeedMPS, 4) - (Math.pow(gravity, 2) * Math.pow(horizontalDistance.in(Meter), 2)) + (2 * gravity * Math.pow(launchSpeedMPS, 2) * -heightOffset.in(Meter));

        if (discriminant < 0) {
            return null;
        }

        double square_root = Math.sqrt(discriminant);

        double numerator = (Math.pow(launchSpeedMPS, 2)) + square_root;
        double denominator = gravity * horizontalDistance.in(Meter);

        return Radians.of(Math.atan(numerator / denominator));
    }

    /**
     * Simulates the launch of a projectile using runge kutta 4.
     * 
     * @param launchSpeed The launch velocity of the projectile as a {@link LinearVelocity}
     * @param launchPitch The launch pitch of the projectile as a {@link Angle}
     * @param launchYaw The launch yaw of the projectile as a {@link Angle}
     * @param robotVelocity The velocity of the robot as a {@link Translation3d} in Meters/Second
     * @param targetPosition The target position in field relative coordinates centered at the robot in {@link Translation3d} in Meter
     * @param tps The ticks per second of the simulation
     * @return A {@link Translation3d} array of the last two positions of the projectile
     */
    public Translation3d[] simulateLaunch(LinearVelocity launchSpeed, Angle launchPitch, Angle launchYaw, AngularVelocity launchAngularPitch, AngularVelocity launchAngularYaw, Translation2d robotVelocity, Translation3d targetPosition, Angle targetDirectAngle, int tps) {
        
        double noteVerticalOffset = Math.sin(launchPitch.in(Radians)) * Constants.TurretConstants.TURRET_PIVOT_FUEL_OFFSET.in(Meter);
        double noteForwardOffset = Math.cos(launchPitch.in(Radians)) * Constants.TurretConstants.TURRET_PIVOT_FUEL_OFFSET.in(Meter);

        double noteXOffset = Math.cos(launchYaw.in(Radians)) * noteForwardOffset;
        double noteYOffset = Math.sin(launchYaw.in(Radians)) * noteForwardOffset;

        double posX = Constants.TurretConstants.TURRET_PIVOT_OFFSET.getX() + noteXOffset;
        double posY = Constants.TurretConstants.TURRET_PIVOT_OFFSET.getY() + noteYOffset;
        double posZ = Constants.TurretConstants.TURRET_PIVOT_OFFSET.getZ() + noteVerticalOffset;

        double velX = (launchSpeed.in(MetersPerSecond) * Math.cos(launchPitch.in(Radians)) * Math.cos(launchYaw.in(Radians))) + robotVelocity.getX();
        double velY = (launchSpeed.in(MetersPerSecond) * Math.cos(launchPitch.in(Radians)) * Math.sin(launchYaw.in(Radians))) + robotVelocity.getY();
        double velZ = launchSpeed.in(MetersPerSecond) * Math.sin(launchPitch.in(Radians));

        double deltaTime = 1.0 / tps;

        double dragConstant = 0.5 * dragCoefficient * fluidDensity * crossSectionArea;
        double magnusConstant = 0.5 * liftCoefficient * fluidDensity * crossSectionArea * projectileRadius;
        double rotDragFactor = (0.5 * fluidDensity * rotDragCoefficient * crossSectionArea * projectileRadius) / momentOfInertia;

        double pitchSpinAxisYaw = launchYaw.in(Radians) + (Math.PI / 2.0);

        double angX = launchAngularPitch.in(RadiansPerSecond) * Math.cos(pitchSpinAxisYaw);
        double angY = launchAngularPitch.in(RadiansPerSecond) * Math.sin(pitchSpinAxisYaw);
        double angZ = launchAngularYaw.in(RadiansPerSecond);

        //double horizontalDistance = Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2));

        double prevX = posX;
        double prevY = posY;
        double prevZ = posZ;

        double k1vx, k1vy, k1vz, k1ax, k1ay, k1az, k1wx, k1wy, k1wz, k1alphax, k1alphay, k1alphaz;
        double k2vx, k2vy, k2vz, k2ax, k2ay, k2az, k2wx, k2wy, k2wz, k2alphax, k2alphay, k2alphaz;
        double k3vx, k3vy, k3vz, k3ax, k3ay, k3az, k3wx, k3wy, k3wz, k3alphax, k3alphay, k3alphaz;
        double k4vx, k4vy, k4vz, k4ax, k4ay, k4az, k4wx, k4wy, k4wz, k4alphax, k4alphay, k4alphaz;

        double magVel, magAngVel, dragFactor;

        double magnusX, magnusY, magnusZ;

        for (int step = 0; step < 60 * tps; step++) {
            prevX = posX;
            prevY = posY;
            prevZ = posZ;

            // K1
            k1vx = velX;
            k1vy = velY;
            k1vz = velZ;
            k1wx = angX;
            k1wy = angY;
            k1wz = angZ;
            magVel = Math.sqrt(k1vx * k1vx + k1vy * k1vy + k1vz * k1vz);
            magAngVel = Math.sqrt(k1wx * k1wx + k1wy * k1wy + k1wz * k1wz);
            dragFactor = -dragConstant * magVel;

            magnusX = magnusConstant * (k1wy * k1vz - k1wz * k1vy);
            magnusY = magnusConstant * (k1wz * k1vx - k1wx * k1vz);
            magnusZ = magnusConstant * (k1wx * k1vy - k1wy * k1vx);

            k1ax = (((k1vx * dragFactor) + magnusX) / mass);
            k1ay = (((k1vy * dragFactor) + magnusY) / mass);
            k1az = (((k1vz * dragFactor) + magnusZ) / mass) - gravity;
            
            k1alphax = -rotDragFactor * k1wx * magAngVel;
            k1alphay = -rotDragFactor * k1wy * magAngVel;
            k1alphaz = -rotDragFactor * k1wz * magAngVel;

            // K2
            k2vx = velX + (0.5 * k1ax * deltaTime);
            k2vy = velY + (0.5 * k1ay * deltaTime);
            k2vz = velZ + (0.5 * k1az * deltaTime);
            k2wx = angX + (0.5 * k1alphax * deltaTime);
            k2wy = angY + (0.5 * k1alphay * deltaTime);
            k2wz = angZ + (0.5 * k1alphaz * deltaTime);
            magVel = Math.sqrt(k2vx * k2vx + k2vy * k2vy + k2vz * k2vz);
            magAngVel = Math.sqrt(k2wx * k2wx + k2wy * k2wy + k2wz * k2wz);
            dragFactor = -dragConstant * magVel;

            magnusX = magnusConstant * (k2wy * k2vz - k2wz * k2vy);
            magnusY = magnusConstant * (k2wz * k2vx - k2wx * k2vz);
            magnusZ = magnusConstant * (k2wx * k2vy - k2wy * k2vx);

            k2ax = (((k2vx * dragFactor) + magnusX) / mass);
            k2ay = (((k2vy * dragFactor) + magnusY) / mass);
            k2az = (((k2vz * dragFactor) + magnusZ) / mass) - gravity;
            
            k2alphax = -rotDragFactor * k2wx * magAngVel;
            k2alphay = -rotDragFactor * k2wy * magAngVel;
            k2alphaz = -rotDragFactor * k2wz * magAngVel;

            // K3
            k3vx = velX + (0.5 * k2ax * deltaTime);
            k3vy = velY + (0.5 * k2ay * deltaTime);
            k3vz = velZ + (0.5 * k2az * deltaTime);
            k3wx = angX + (0.5 * k2alphax * deltaTime);
            k3wy = angY + (0.5 * k2alphay * deltaTime);
            k3wz = angZ + (0.5 * k2alphaz * deltaTime);
            magVel = Math.sqrt(k3vx * k3vx + k3vy * k3vy + k3vz * k3vz);
            magAngVel = Math.sqrt(k3wx * k3wx + k3wy * k3wy + k3wz * k3wz);
            dragFactor = -dragConstant * magVel;

            magnusX = magnusConstant * (k3wy * k3vz - k3wz * k3vy);
            magnusY = magnusConstant * (k3wz * k3vx - k3wx * k3vz);
            magnusZ = magnusConstant * (k3wx * k3vy - k3wy * k3vx);

            k3ax = (((k3vx * dragFactor) + magnusX) / mass);
            k3ay = (((k3vy * dragFactor) + magnusY) / mass);
            k3az = (((k3vz * dragFactor) + magnusZ) / mass) - gravity;
            
            k3alphax = -rotDragFactor * k3wx * magAngVel;
            k3alphay = -rotDragFactor * k3wy * magAngVel;
            k3alphaz = -rotDragFactor * k3wz * magAngVel;

            // K4
            k4vx = velX + (k3ax * deltaTime);
            k4vy = velY + (k3ay * deltaTime);
            k4vz = velZ + (k3az * deltaTime);
            k4wx = angX + (k3alphax * deltaTime);
            k4wy = angY + (k3alphay * deltaTime);
            k4wz = angZ + (k3alphaz * deltaTime);
            magVel = Math.sqrt(k4vx * k4vx + k4vy * k4vy + k4vz * k4vz);
            magAngVel = Math.sqrt(k4wx * k4wx + k4wy * k4wy + k4wz * k4wz);
            dragFactor = -dragConstant * magVel;

            magnusX = magnusConstant * (k4wy * k4vz - k4wz * k4vy);
            magnusY = magnusConstant * (k4wz * k4vx - k4wx * k4vz);
            magnusZ = magnusConstant * (k4wx * k4vy - k4wy * k4vx);

            k4ax = (((k4vx * dragFactor) + magnusX) / mass);
            k4ay = (((k4vy * dragFactor) + magnusY) / mass);
            k4az = (((k4vz * dragFactor) + magnusZ) / mass) - gravity;
            
            k4alphax = -rotDragFactor * k4wx * magAngVel;
            k4alphay = -rotDragFactor * k4wy * magAngVel;
            k4alphaz = -rotDragFactor * k4wz * magAngVel;

            posX += (k1vx + 2 * k2vx + 2 * k3vx + k4vx) / 6.0 * deltaTime;
            posY += (k1vy + 2 * k2vy + 2 * k3vy + k4vy) / 6.0 * deltaTime;
            posZ += (k1vz + 2 * k2vz + 2 * k3vz + k4vz) / 6.0 * deltaTime;

            velX += (k1ax + 2 * k2ax + 2 * k3ax + k4ax) / 6.0 * deltaTime;
            velY += (k1ay + 2 * k2ay + 2 * k3ay + k4ay) / 6.0 * deltaTime;
            velZ += (k1az + 2 * k2az + 2 * k3az + k4az) / 6.0 * deltaTime;

            angX += (k1alphax + 2 * k2alphax + 2 * k3alphax + k4alphax) / 6.0 * deltaTime;
            angY += (k1alphay + 2 * k2alphay + 2 * k3alphay + k4alphay) / 6.0 * deltaTime;
            angZ += (k1alphaz + 2 * k2alphaz + 2 * k3alphaz + k4alphaz) / 6.0 * deltaTime;
            
            if (velZ < 0 && posZ < targetPosition.getZ()) {
                break;
            }
            
            /*if (Math.sqrt(Math.pow(posX, 2) + Math.pow(posY, 2)) >= horizontalDistance) {
                break;
            }*/
        }
        return new Translation3d[]{
            new Translation3d(prevX, prevY, prevZ),
            new Translation3d(posX, posY , posZ)
        };
    }

    /**
     * Interpolates the position of the projectile between two points
     * 
     * @param path A double array that contains the last two positions of the projectile as a {@link Translation3d} in Meters
     * @param horizontalDistance The horizontal distance of the target as a {@link Distance}
     * @return The interpolated positon of the projectile as a {@link Translation3d} in Meters
     */
    private Translation3d interpolatePosition(Translation3d[] path, Translation3d targetPosition, Angle targetDirectAngle) {
        //double horizontalDistance1 = Math.sqrt(Math.pow(path[0].getX(), 2) + Math.pow(path[0].getY(), 2));
        //double horizontalDistance2 = Math.sqrt(Math.pow(path[1].getX(), 2) + Math.pow(path[1].getY(), 2));
        double zHeight1 = path[0].getZ();
        double zHeight2 = path[1].getZ();

        if (Math.abs(zHeight2 - zHeight1) < 1e-9) {
            return path[0];
        }

        double weight = (targetPosition.getZ() - zHeight1) / (zHeight2 - zHeight1);

        return new Translation3d(
            path[0].getX() + ((path[1].getX() - path[0].getX()) * weight),
            path[0].getY() + ((path[1].getY() - path[0].getY()) * weight),
            path[0].getZ() + ((path[1].getZ() - path[0].getZ()) * weight)
        );
    }

    /**
     * Calculates the error of the projectiles landing point
     * 
     * @param launchSpeed The launch speed of the projectile as a {@link LinearVelocity}
     * @param launchPitch The launch pitch of the projectile as a {@link Angle}
     * @param launchYaw The launch yaw of the projectile as a {@link Angle}
     * @param robotVelocity The field relative velocity of the robot as a {@link LinearVelocity} in Meters/Second
     * @param targetPosition The field relative position of the target centered at the robot
     * @param targetDirectAngle The direct yaw {@link Angle} towards the target
     * @param horizontalDistance The {@link Distance} from the robot to the target 
     * @param tps The ticks per second of the simulation
     * @return The error in the simulation in the order:
     *  <ul>
     *       <li>Height error</li>
     *       <li>Yaw error</li>
     *       <li>Vertical velocity</li>
     *  </ul>
     */
    private double[] calculateLaunchError(LinearVelocity launchSpeed, Angle launchPitch, Angle launchYaw, AngularVelocity launchAngularPitch, AngularVelocity launchAngularYaw, Translation2d robotVelocity, Translation3d targetPosition, Angle targetDirectAngle, Distance horizontalDistance, int tps) {
        Translation3d[] path = simulateLaunch(
            launchSpeed,
            launchPitch,
            launchYaw,
            launchAngularPitch,
            launchAngularYaw,
            robotVelocity,
            targetPosition,
            targetDirectAngle,
            tps
        );
        Translation3d crossOverPoint = interpolatePosition(path, targetPosition, targetDirectAngle);

        double landing_yaw = Math.atan2(crossOverPoint.getY(), crossOverPoint.getX());

        double verticalVelocity = (path[1].getZ() - path[0].getZ()) * tps;

        double targetHorizontalDist = Math.sqrt(
            targetPosition.getX() * targetPosition.getX() + 
            targetPosition.getY() * targetPosition.getY()
        );
        
        // Create unit vector in direction of target (horizontal plane only)
        double unitX = targetPosition.getX() / targetHorizontalDist;
        double unitY = targetPosition.getY() / targetHorizontalDist;
        
        // Project crossover point onto target direction vector (dot product)
        double projectedDistance = (crossOverPoint.getX() * unitX) + 
                                  (crossOverPoint.getY() * unitY);
        
        // Signed error: positive = overshot, negative = undershot
        double distanceError = projectedDistance - targetHorizontalDist;

        if (path[0].getZ() < targetPosition.getZ()) {
            distanceError = -1;
        }

        return new double[]{
            distanceError,
            MathUtil.angleModulus(landing_yaw - targetDirectAngle.in(Radians)),
            verticalVelocity
        };
    }

    public LinearVelocity convertShooterSpeedToVelocity(AngularVelocity angularSpeed, Distance radius, double efficency) {
        return MetersPerSecond.of((angularSpeed.in(RadiansPerSecond) * radius.in(Meter)) * efficency);
    }

    public AngularVelocity convertVelocityToShooterSpeed(LinearVelocity linearVelocity, Distance radius, double efficency) {
        return RadiansPerSecond.of((linearVelocity.in(MetersPerSecond) / radius.in(Meter)) / efficency);
    }

    /**
     * 
     * @param startLaunchSpeed The starting launch speed of the projectile as a {@link LinearVelocity}
     * @param launchAngularYaw The angular yaw of the projectile as a {@link AngularVelocity}
     * @param robotVelocity The field relative velocity of the robot as a {@link Translation2d} in Meters/Second
     * @param targetPosition The robot relative position of the target (Rotation is field relative) as a {@link Translation3d} in Meters
     * @param maxSteps The max amount of optimization steps (It can exit early if the error gets below a threshold)
     * @param tps The ticks per second that physics will be calculated at
     * @return The target solution
     */
    public TargetSolution calculateLaunchAngleSimulation(LinearVelocity startLaunchSpeed, AngularVelocity launchAngularYaw, Translation2d robotVelocity, Translation3d targetPosition, int maxSteps, int tps) {
        
        Distance horizontalDistance = Meter.of(Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2)));

        double targetDirectAngle = Math.atan2(targetPosition.getY(), targetPosition.getX());

        Angle launchAnglePitch1Temp = calculateLaunchPitchIdeal(startLaunchSpeed, horizontalDistance, Meter.of(targetPosition.getZ() - Constants.TurretConstants.TURRET_PIVOT_OFFSET.getZ()));

        if (launchAnglePitch1Temp == null) {
            return new TargetSolution(TargetErrorCode.IDEAL_PITCH, MetersPerSecond.of(0), Radians.of(0.0), Radians.of(0.0), new TargetDebug(0, 0, 0, 0));
        }

        double pitchLimitUpper = Constants.TurretConstants.TURRET_UPPER_LIMIT.in(Radians);
        double pitchLimitLower = Constants.TurretConstants.TURRET_LOWER_LIMIT.in(Radians);

        double speedLimitUpper = convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5).in(MetersPerSecond);
        double speedLimitLower = convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MIN_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5).in(MetersPerSecond);

        double launchPitch = launchAnglePitch1Temp.in(Radians);
        double launchSpeed = speedLimitUpper;
        double launchYaw = targetDirectAngle;

        double[] error = calculateLaunchError(MetersPerSecond.of(launchSpeed), Radians.of(launchPitch), Radians.of(launchYaw), RadiansPerSecond.of(-(launchSpeed / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        double[] pitchError = calculateLaunchError(MetersPerSecond.of(launchSpeed), Radians.of(launchPitch - 0.05), Radians.of(launchYaw), RadiansPerSecond.of(-(launchSpeed / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        double[] yawError = calculateLaunchError(MetersPerSecond.of(launchSpeed), Radians.of(launchPitch), Radians.of(launchYaw + 0.05), RadiansPerSecond.of(-((launchSpeed) / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        double pitchSensitivity = (pitchError[0] - error[0]) / -0.05;
        double yawSensitivity = (yawError[1] - error[1]) / 0.05;

        double pitchDelta = 0;
        double yawDelta = 0;

        double lastPitch = launchPitch;
        double lastYaw = launchYaw;
        double[] lastError = error;

        int pitchYawSteps = 0;
        for (pitchYawSteps = 0; pitchYawSteps < 30; pitchYawSteps++) {

            if (Math.abs(pitchSensitivity) < 1e-5) {
                pitchDelta = 0;
            } else {
                pitchDelta = MathUtil.clamp((error[0] / pitchSensitivity), -0.1, 0.1);
            }


            if (Math.abs(yawSensitivity) < 1e-5) {
                yawDelta = 0;
            } else {
                yawDelta = error[1] / Math.max(((pitchYawSteps + 2) / 2.5), 1);
            }

            lastPitch = launchPitch;
            lastYaw = launchYaw;

            launchPitch -= pitchDelta;
            launchYaw -= yawDelta;

            launchYaw = MathUtil.angleModulus(launchYaw);

            if (pitchYawSteps > 15) {
                if (launchPitch > pitchLimitUpper) launchPitch = pitchLimitUpper;
                if (launchPitch < pitchLimitLower) launchPitch = pitchLimitLower;
            }
            double[] newError = calculateLaunchError(
                MetersPerSecond.of(launchSpeed), 
                Radians.of(launchPitch), 
                Radians.of(launchYaw), 
                RadiansPerSecond.of(-(launchSpeed / projectileRadius)), 
                launchAngularYaw, 
                robotVelocity, 
                targetPosition, 
                Radians.of(targetDirectAngle), 
                horizontalDistance, 
                tps
            );

            double actualPitchDelta = launchPitch - lastPitch;
            double actualYawDelta = launchYaw - lastYaw;
            
            if (Math.abs(actualPitchDelta) > 1e-5) {
                pitchSensitivity = (newError[0] - lastError[0]) / actualPitchDelta;
            }
            /*
            if (Math.abs(actualYawDelta) > 1e-5) {
                yawSensitivity = (newError[1] - lastError[1]) / actualYawDelta;
            }
            */

            lastError = error; 
            error = newError;
        }

        int speedSteps = 0;
        if (Math.abs(error[0]) > 10000000) {
            double speedChangeUpperLimit = speedLimitUpper;
            double speedChangeLowerLimit = speedLimitLower;
            for (speedSteps = 0; speedSteps < 10; speedSteps++) {
                launchSpeed = ((speedChangeUpperLimit - speedChangeLowerLimit) / 2.0) + speedChangeLowerLimit;

                error = calculateLaunchError(
                    MetersPerSecond.of(launchSpeed), 
                    Radians.of(launchPitch), 
                    Radians.of(launchYaw), 
                    RadiansPerSecond.of(-(launchSpeed / projectileRadius)), 
                    launchAngularYaw, 
                    robotVelocity, 
                    targetPosition, 
                    Radians.of(targetDirectAngle), 
                    horizontalDistance, 
                    tps
                );

                if (error[0] > 0) {
                    speedChangeUpperLimit = launchSpeed;
                } else {
                    speedChangeLowerLimit = launchSpeed;
                }
            }
        }

        TargetErrorCode solutionFound = TargetErrorCode.NONE;
        if (Math.abs(launchYaw) > (Math.PI * 3)) {
            solutionFound = TargetErrorCode.EXCESSIVE_YAW;
        } else if (launchPitch > Constants.TurretConstants.TURRET_UPPER_LIMIT.in(Radians)) {
            solutionFound = TargetErrorCode.PITCH_UPPER_LIMIT;
        } else if (launchPitch < Constants.TurretConstants.TURRET_LOWER_LIMIT.in(Radians)) {
            solutionFound = TargetErrorCode.PITCH_LOWER_LIMIT;
        } else if (Math.abs(error[0]) > 0.1) {
            solutionFound = TargetErrorCode.HEIGHT_ERROR_HIGH;
        } else if (Math.abs(error[1]) > 0.1) {
            solutionFound = TargetErrorCode.YAW_ERROR_HIGH;
        } else if (launchSpeed > speedLimitUpper) {
            solutionFound = TargetErrorCode.SPEED_UPPER_LIMIT;
        } else if (launchSpeed < speedLimitLower) {
            solutionFound = TargetErrorCode.SPEED_LOWER_LIMIT;
        }

        return new TargetSolution(
            solutionFound,
            MetersPerSecond.of(launchSpeed),
            Radians.of(launchPitch),
            Radians.of(launchYaw), 
            new TargetDebug(pitchYawSteps, speedSteps, error[0], error[1])
        );
    }

    /**
     * 
     * @param startLaunchSpeed The starting launch speed of the projectile as a {@link LinearVelocity}
     * @param launchAngularYaw The angular yaw of the projectile as a {@link AngularVelocity}
     * @param robotVelocity The field relative velocity of the robot as a {@link Translation2d} in Meters/Second
     * @param targetPosition The robot relative position of the target (Rotation is field relative) as a {@link Translation3d} in Meters
     * @param maxSteps The max amount of optimization steps (It can exit early if the error gets below a threshold)
     * @param tps The ticks per second that physics will be calculated at
     * @return The target solution
     */
    public TargetSolution calculateLaunchAngleSimulationOld2(LinearVelocity startLaunchSpeed, AngularVelocity launchAngularYaw, Translation2d robotVelocity, Translation3d targetPosition, int maxSteps, int tps) {
        double pitchWeight = 0.5;

        Distance horizontalDistance = Meter.of(Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2)));

        double targetDirectAngle = Math.atan2(targetPosition.getY(), targetPosition.getX());

        Angle launchAnglePitch1Temp = calculateLaunchPitchIdeal(startLaunchSpeed, horizontalDistance, Meter.of(targetPosition.getZ() - Constants.TurretConstants.TURRET_PIVOT_OFFSET.getZ()));

        if (launchAnglePitch1Temp == null) {
            return new TargetSolution(TargetErrorCode.IDEAL_PITCH, MetersPerSecond.of(0), Radians.of(0.0), Radians.of(0.0), new TargetDebug(0, 0, 0, 0));
        }

        double pitchLimitUpper = Constants.TurretConstants.TURRET_UPPER_LIMIT.in(Radians);
        double pitchLimitLower = Constants.TurretConstants.TURRET_LOWER_LIMIT.in(Radians);

        double speedLimitUpper = convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5).in(MetersPerSecond);
        double speedLimitLower = convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MIN_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5).in(MetersPerSecond);

        double launchPitch = launchAnglePitch1Temp.in(Radians);
        double launchSpeed = startLaunchSpeed.in(MetersPerSecond);

        double[] error = calculateLaunchError(MetersPerSecond.of(launchSpeed), Radians.of(launchPitch), Radians.of(0), RadiansPerSecond.of(-(launchSpeed / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        double[] pitchError = calculateLaunchError(MetersPerSecond.of(launchSpeed), Radians.of(launchPitch + 0.05), Radians.of(0), RadiansPerSecond.of(-(launchSpeed / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        double[] speedError = calculateLaunchError(MetersPerSecond.of(launchSpeed + 0.1), Radians.of(launchPitch), Radians.of(0), RadiansPerSecond.of(-((launchSpeed + 0.1) / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        double pitchSensitivity = (pitchError[0] - error[0]) / 0.05;
        double speedSensitivity = (speedError[0] - error[0]) / 0.1;

        double pitchDelta = 0;
        double speedDelta = 0;

        int steps = 0;
        for (steps = 0; steps < maxSteps; steps++) {

            if (Math.abs(error[0]) < 0.01) break;


            double weightedPitchSensitivity = pitchSensitivity / pitchWeight;

            double effectivePitchSens = weightedPitchSensitivity;
            double effectiveSpeedSens = speedSensitivity;

            if ((launchPitch >= pitchLimitUpper && error[0] * pitchSensitivity < 0) || 
                (launchPitch <= pitchLimitLower && error[0] * pitchSensitivity > 0)) {
                effectivePitchSens = 0.1;
            }

            if ((launchSpeed >= speedLimitUpper && error[0] * speedSensitivity < 0) || 
                (launchSpeed <= speedLimitLower && error[0] * speedSensitivity > 0)) {
                effectiveSpeedSens = 0.1;
            }

            double l2Norm = effectivePitchSens * effectivePitchSens + effectiveSpeedSens * effectiveSpeedSens;
            
            if (l2Norm < 1e-9) break;

            pitchDelta = ((-error[0] / l2Norm) * effectivePitchSens) / pitchWeight;
            speedDelta = -error[0] * (effectiveSpeedSens / l2Norm);

            double t = Math.sqrt(pitchDelta * pitchDelta + speedDelta * speedDelta);
            if (t > 2) {
                pitchDelta /= 0.5 * t;
                speedDelta /= 0.5 * t;
            }

            double oldPitch = launchPitch;
            double oldSpeed = launchSpeed;

            launchPitch = MathUtil.clamp(launchPitch + pitchDelta, pitchLimitLower, pitchLimitUpper);
            launchSpeed = MathUtil.clamp(launchSpeed + speedDelta, speedLimitLower, speedLimitUpper);

            pitchDelta = launchPitch - oldPitch;
            speedDelta = launchSpeed - oldSpeed;

            double[] newError = calculateLaunchError(MetersPerSecond.of(launchSpeed), Radians.of(launchPitch), Radians.of(0), RadiansPerSecond.of(-(launchSpeed / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

            double errorDelta = newError[0] - error[0];

            double predictedDelta = (pitchSensitivity * pitchDelta) + (speedSensitivity * speedDelta);

            double discrepancy = errorDelta - predictedDelta;

            double distanceDelta = pitchDelta * pitchDelta + speedDelta * speedDelta;

            //System.out.println(newError + " [" + launchPitch + ", " + launchSpeed + "] " + pitchDelta + " " + speedDelta);

            if (distanceDelta > 1e-12) {
                pitchSensitivity += (discrepancy * pitchDelta) / distanceDelta;
                speedSensitivity += (discrepancy * speedDelta) / distanceDelta;
            } else {
                break;
            }

            error = newError;
        }

        TargetErrorCode solutionFound = TargetErrorCode.NONE;
        if (Math.abs(0) > (Math.PI * 3)) {
            solutionFound = TargetErrorCode.EXCESSIVE_YAW;
        } else if (launchPitch > Constants.TurretConstants.TURRET_UPPER_LIMIT.in(Radians)) {
            solutionFound = TargetErrorCode.PITCH_UPPER_LIMIT;
        } else if (launchPitch < Constants.TurretConstants.TURRET_LOWER_LIMIT.in(Radians)) {
            solutionFound = TargetErrorCode.PITCH_LOWER_LIMIT;
        } else if (Math.abs(error[0]) > 0.1) {
            solutionFound = TargetErrorCode.HEIGHT_ERROR_HIGH;
        } else if (Math.abs(error[1]) > 0.1) {
            solutionFound = TargetErrorCode.YAW_ERROR_HIGH;
        } else if (launchSpeed > speedLimitUpper) {
            solutionFound = TargetErrorCode.SPEED_UPPER_LIMIT;
        } else if (launchSpeed < speedLimitLower) {
            solutionFound = TargetErrorCode.SPEED_LOWER_LIMIT;
        }

        return new TargetSolution(
            solutionFound,
            MetersPerSecond.of(launchSpeed),
            Radians.of(launchPitch),
            Radians.of(0), 
            new TargetDebug(steps, 0, error[0], error[1])
        );
    }

    /**
     * 
     * @param startLaunchSpeed The starting launch speed of the projectile as a {@link LinearVelocity}
     * @param launchAngularYaw The angular yaw of the projectile as a {@link AngularVelocity}
     * @param robotVelocity The field relative velocity of the robot as a {@link Translation2d} in Meters/Second
     * @param targetPosition The robot relative position of the target (Rotation is field relative) as a {@link Translation3d} in Meters
     * @param maxSteps The max amount of optimization steps (It can exit early if the error gets below a threshold)
     * @param tps The ticks per second that physics will be calculated at
     * @return The target solution
     */
    public TargetSolution calculateLaunchAngleSimulationOld(LinearVelocity startLaunchSpeed, AngularVelocity launchAngularYaw, Translation2d robotVelocity, Translation3d targetPosition, int maxSteps, int tps) {
        Distance horizontalDistance = Meter.of(Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2)));

        double targetDirectAngle = Math.atan2(targetPosition.getY(), targetPosition.getX());

        Angle launchAnglePitch1Temp = calculateLaunchPitchIdeal(startLaunchSpeed, horizontalDistance, Meter.of(targetPosition.getZ() - Constants.TurretConstants.TURRET_PIVOT_OFFSET.getZ()));

        if (launchAnglePitch1Temp == null) {
            return new TargetSolution(TargetErrorCode.IDEAL_PITCH, MetersPerSecond.of(0), Radians.of(0.0), Radians.of(0.0), new TargetDebug(0, 0, 0, 0));
        }

        double pitchLimitUpper = Constants.TurretConstants.TURRET_UPPER_LIMIT.in(Radians);
        double pitchLimitLower = Constants.TurretConstants.TURRET_LOWER_LIMIT.in(Radians);

        double speedLimitUpper = convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5).in(MetersPerSecond);
        double speedLimitLower = convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MIN_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5).in(MetersPerSecond);


        double launchAnglePitch1 = launchAnglePitch1Temp.in(Radians);

        double launchAnglePitch2 = launchAnglePitch1 + 0.005;
        double launchAngleYaw1 = targetDirectAngle - 0.01;
        double launchAngleYaw2 = targetDirectAngle + 0.01;
        double launchSpeed1 = startLaunchSpeed.in(MetersPerSecond);
        double launchSpeed2 = launchSpeed1 + 1;

        // Height Error, Yaw Error
        double[] launchError1 = calculateLaunchError(MetersPerSecond.of(launchSpeed1), Radians.of(launchAnglePitch1), Radians.of(launchAngleYaw1), RadiansPerSecond.of(-(launchSpeed1 / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);
        double[] launchError2 = calculateLaunchError(MetersPerSecond.of(launchSpeed1), Radians.of(launchAnglePitch2), Radians.of(launchAngleYaw2), RadiansPerSecond.of(-(launchSpeed1 / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        boolean modifingSpeed = false;

        int steps;
        for (steps = 0; steps < maxSteps; steps++) {
            

            double maxStep = Math.toRadians(10); 

            double weightYaw = 0;
            if (Math.abs(launchError1[1]) > 1e-7 && Math.abs(launchError1[1] - launchError2[1]) > 1e-7) {
                weightYaw = (launchAngleYaw1 - launchAngleYaw2) / (launchError1[1] - launchError2[1]);
            }
            launchAngleYaw2 = launchAngleYaw1;
            launchAngleYaw1 -= MathUtil.clamp((launchError1[1] * weightYaw), -maxStep, maxStep);



            double weightPitch = 0;
            double weightSpeed = 0;
            if (Math.abs(launchError1[0]) > 1e-7 && Math.abs(launchError1[0] - launchError2[0]) > 1e-7) {
                weightPitch = (launchAnglePitch1 - launchAnglePitch2) / (launchError1[0] - launchError2[0]);
                weightSpeed = (launchSpeed1 - launchSpeed2) / (launchError1[0] - launchError2[0]);
            }


            if (modifingSpeed) {
                launchSpeed2 = launchSpeed1;
                launchSpeed1 -= MathUtil.clamp((launchError1[0] * weightSpeed), -3, 3);
                launchSpeed1 = MathUtil.clamp(
                    launchSpeed1,
                    speedLimitLower,
                    speedLimitUpper
                );

            } else {
                launchAnglePitch2 = launchAnglePitch1;
                launchAnglePitch1 -= MathUtil.clamp((launchError1[0] * weightPitch), -maxStep, maxStep);
            }
            //System.out.println(launchAnglePitch2 - launchAnglePitch1);
            launchError2 = launchError1;
            if (modifingSpeed == false) {
                if (steps >= 6 && Math.abs(launchAnglePitch2 - launchAnglePitch1) < 2e-5) {
                    modifingSpeed = true;

                    launchAnglePitch1 = MathUtil.clamp(
                        launchAnglePitch1,
                        pitchLimitLower,
                        pitchLimitUpper
                    );
                    launchAnglePitch2 = launchAnglePitch1;
                    //launchError1 = new double[] {0.01, launchError1[1], launchError1[2]};
                }
            }
            

            // Height Error, Yaw Error, Vertical Speed
            launchError1 = calculateLaunchError(MetersPerSecond.of(launchSpeed1), Radians.of(launchAnglePitch1), Radians.of(launchAngleYaw1), RadiansPerSecond.of(-(launchSpeed1 / projectileRadius) ), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

            if (Math.abs(launchError1[0]) < 2e-3 && Math.abs(launchError1[1]) < 2e-3 && launchError1[2] < 0 && steps >= 10) {
                break;
            }
        }
        TargetErrorCode solutionFound = TargetErrorCode.NONE;
        if (Math.abs(launchAngleYaw1) > (Math.PI * 3)) {
            solutionFound = TargetErrorCode.EXCESSIVE_YAW;
        } else if (launchAnglePitch1 > Constants.TurretConstants.TURRET_UPPER_LIMIT.in(Radians)) {
            solutionFound = TargetErrorCode.PITCH_UPPER_LIMIT;
        } else if (launchAnglePitch1 < Constants.TurretConstants.TURRET_LOWER_LIMIT.in(Radians)) {
            solutionFound = TargetErrorCode.PITCH_LOWER_LIMIT;
        } else if (Math.abs(launchError1[0]) > 0.1) {
            solutionFound = TargetErrorCode.HEIGHT_ERROR_HIGH;
        } else if (Math.abs(launchError1[1]) > 0.1) {
            solutionFound = TargetErrorCode.YAW_ERROR_HIGH;
        } else if (launchSpeed1 > speedLimitUpper) {
            solutionFound = TargetErrorCode.SPEED_UPPER_LIMIT;
        } else if (launchSpeed1 < speedLimitLower) {
            solutionFound = TargetErrorCode.SPEED_LOWER_LIMIT;
        }

        return new TargetSolution(
            solutionFound, MetersPerSecond.of(launchSpeed1),
            Radians.of(launchAnglePitch1),
            Radians.of(MathUtil.inputModulus(launchAngleYaw1,0, 2 * Math.PI)),
            new TargetDebug(steps, 0, launchError1[0], launchError1[1])
        );
    }
}