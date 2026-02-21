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
     *       <li>FORWARD_ERROR_HIGH = Forward error exceeds 0.1 meters</li>
     *       <li>RIGHT_ERROR_HIGH = Right error exceeds 0.1 meters</li>
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
        FORWARD_ERROR_HIGH,
        RIGHT_ERROR_HIGH,
        SPEED_UPPER_LIMIT,
        SPEED_LOWER_LIMIT
    }

    public record TargetDebug (
        int pitchYawSteps,
        double forwardError,
        double rightError
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
        double rotDragFactor = (0.5 * fluidDensity * rotDragCoefficient * crossSectionArea * Math.pow(projectileRadius, 3)) / momentOfInertia;

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
     *       <li>Forward error</li>
     *       <li>Right error</li>
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

        double verticalVelocity = (path[1].getZ() - path[0].getZ()) * tps;

        double targetHorizontalDist = Math.sqrt(
            targetPosition.getX() * targetPosition.getX() + 
            targetPosition.getY() * targetPosition.getY()
        );
        

        double unitX = targetPosition.getX() / targetHorizontalDist;
        double unitY = targetPosition.getY() / targetHorizontalDist;
        
        double projectedDistanceForward = (crossOverPoint.getX() * unitX) + 
                                  (crossOverPoint.getY() * unitY);
        
        double projectedDistanceRight = (crossOverPoint.getX() * unitY) + 
                                  (crossOverPoint.getY() * -unitX);
        
        // Signed error: positive = overshot, negative = undershot
        double forwardError = projectedDistanceForward - targetHorizontalDist;

        if (path[0].getZ() < targetPosition.getZ()) {
            forwardError = -1;
        }

        return new double[]{
            forwardError,
            projectedDistanceRight,
            verticalVelocity
        };
    }

    public LinearVelocity convertShooterSpeedToVelocity(AngularVelocity angularSpeed, Distance radius, double efficency) {
        return MetersPerSecond.of((angularSpeed.in(RadiansPerSecond) * radius.in(Meter)) * efficency);
    }

    public AngularVelocity convertVelocityToShooterSpeed(LinearVelocity linearVelocity, Distance radius, double efficency) {
        return RadiansPerSecond.of((linearVelocity.in(MetersPerSecond) / radius.in(Meter)) / efficency);
    }

    public LinearVelocity estimateShootingVelocity(Translation2d targetPosition, LinearVelocity speedLimitUpper) {
        double targetDistance = targetPosition.getNorm();

        double percent = MathUtil.inverseInterpolate(0, 6, targetDistance);

        return MetersPerSecond.of(
            MathUtil.interpolate(speedLimitUpper.in(MetersPerSecond) * 0.4, speedLimitUpper.in(MetersPerSecond), percent)
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
    public TargetSolution calculateLaunchAngleSimulation(LinearVelocity startLaunchSpeed, AngularVelocity launchAngularYaw, Translation2d robotVelocity, Translation3d targetPosition, int maxSteps, int tps) {
        
        Distance horizontalDistance = Meter.of(Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2)));

        double targetDirectAngle = Math.atan2(targetPosition.getY(), targetPosition.getX());

        Angle launchAnglePitch1Temp = calculateLaunchPitchIdeal(startLaunchSpeed, horizontalDistance, Meter.of(targetPosition.getZ() - Constants.TurretConstants.TURRET_PIVOT_OFFSET.getZ()));

        if (launchAnglePitch1Temp == null) {
            return new TargetSolution(TargetErrorCode.IDEAL_PITCH, MetersPerSecond.of(0), Radians.of(0.0), Radians.of(0.0), new TargetDebug(0, 0, 0));
        }

        double pitchLimitUpper = Constants.TurretConstants.TURRET_UPPER_LIMIT.in(Radians);
        double pitchLimitLower = Constants.TurretConstants.TURRET_LOWER_LIMIT.in(Radians);

        double speedLimitUpper = convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5).in(MetersPerSecond);
        double speedLimitLower = convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MIN_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5).in(MetersPerSecond);

        double launchPitch = launchAnglePitch1Temp.in(Radians);
        double launchSpeed = estimateShootingVelocity(targetPosition.toTranslation2d(), MetersPerSecond.of(speedLimitUpper)).in(MetersPerSecond);
        double launchYaw = targetDirectAngle;

        double perturbation = 0.05;

        double[] error = calculateLaunchError(MetersPerSecond.of(launchSpeed), Radians.of(launchPitch), Radians.of(launchYaw), RadiansPerSecond.of(-(launchSpeed / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        double[] pitchError = calculateLaunchError(MetersPerSecond.of(launchSpeed), Radians.of(launchPitch + perturbation), Radians.of(launchYaw), RadiansPerSecond.of(-(launchSpeed / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        double[] yawError = calculateLaunchError(MetersPerSecond.of(launchSpeed), Radians.of(launchPitch), Radians.of(launchYaw + perturbation), RadiansPerSecond.of(-((launchSpeed) / projectileRadius)), launchAngularYaw, robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        double pitchDelta;
        double yawDelta;

        double pitchForward = (pitchError[0] - error[0]) / perturbation;
        double yawForward = (pitchError[1] - error[1]) / perturbation;
        double pitchRight = (yawError[0] - error[0]) / perturbation;
        double yawRight = (yawError[1] - error[1]) / perturbation;


        int pitchYawSteps = 0;
        for (pitchYawSteps = 0; pitchYawSteps < maxSteps; pitchYawSteps++) {

            double determinant = (pitchForward * yawRight) - (pitchRight * yawForward);

            if (Math.abs(determinant) < 1e-6) {
                break;
            }

            pitchDelta = (yawForward * error[1] - yawRight * error[0]) / determinant;
            yawDelta = (pitchRight * error[0] - pitchForward * error[1]) / determinant;

            pitchDelta = MathUtil.clamp(pitchDelta, -0.1, 0.1);
            yawDelta = MathUtil.clamp(yawDelta, -0.3, 0.3);


            launchPitch += pitchDelta;
            launchYaw += yawDelta;

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

            double deltaForwardError = newError[0] - error[0];
            double deltaRightError = newError[1] - error[1];

            double predictedForwardDelta = (pitchForward * pitchDelta) + (yawForward * yawDelta);
            double predictedRightDelta = (pitchRight * pitchDelta) + (yawRight * yawDelta);

            double predictedActualForward = deltaForwardError - predictedForwardDelta;
            double predictedActualRight = deltaRightError - predictedRightDelta;

            double normSq = (pitchDelta * pitchDelta) + (yawDelta * yawDelta);

            if (normSq > 1e-9) {
                pitchForward += (predictedActualForward * pitchDelta) / normSq;
                yawForward += (predictedActualForward * yawDelta) / normSq;
                pitchRight += (predictedActualRight * pitchDelta) / normSq;
                yawRight += (predictedActualRight * yawDelta) / normSq;
            }

            error = newError;

            if (Math.abs(error[0]) < 0.02 && Math.abs(error[1]) < 0.02) {
                break;
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
            solutionFound = TargetErrorCode.FORWARD_ERROR_HIGH;
        } else if (Math.abs(error[1]) > 0.1) {
            solutionFound = TargetErrorCode.RIGHT_ERROR_HIGH;
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
            new TargetDebug(pitchYawSteps, error[0], error[1])
        );

        
    }
}