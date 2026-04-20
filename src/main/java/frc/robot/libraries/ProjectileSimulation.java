package frc.robot.libraries;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class ProjectileSimulation {
    public final double dragCoefficient;
    public final double rotDragCoefficient; 
    public final double crossSectionArea;
    public final double mass;
    public final double fluidDensity;
    public final double gravity;
    public final double projectileRadius;
    public final double momentOfInertia;
    public final double liftCoefficient;
    public final double fuelPivotOffset;
    public final double shooterWheelRadius;
    public final double pitchLimitLower;
    public final double pitchLimitUpper;
    public final double shooterMinVelocity;
    public final double shooterMaxVelocity;
    public final Translation3d turretOffset;

    public ProjectileSimulation(double dragCoefficient, double rotDragCoefficient, double crossSectionArea, Mass mass, double fluidDensity, LinearAcceleration gravity, double liftCoefficient, Distance fuelPivotOffset, Distance shooterWheelRadius, Translation3d turretOffset, Angle pitchLimitLower, Angle pitchLimitUpper, AngularVelocity shooterMinVelocity, AngularVelocity shooterMaxVelocity) {
        this.dragCoefficient = dragCoefficient;
        this.rotDragCoefficient = rotDragCoefficient;
        this.crossSectionArea = crossSectionArea;
        this.mass = mass.in(Kilogram);
        this.fluidDensity = fluidDensity;
        this.gravity = gravity.in(MetersPerSecondPerSecond);
        this.liftCoefficient = liftCoefficient;
        this.fuelPivotOffset = fuelPivotOffset.in(Meter);
        this.shooterWheelRadius = shooterWheelRadius.in(Meter);
        this.turretOffset = turretOffset;
        this.pitchLimitLower = pitchLimitLower.in(Radians);
        this.pitchLimitUpper = pitchLimitUpper.in(Radians);
        this.shooterMinVelocity = shooterMinVelocity.in(RadiansPerSecond);
        this.shooterMaxVelocity = shooterMaxVelocity.in(RadiansPerSecond);

        this.projectileRadius = Math.sqrt(crossSectionArea / Math.PI);
        this.momentOfInertia = 0.4 * this.mass * Math.pow(this.projectileRadius, 2);

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
        double rightError,
        Time computationTime
    ) {};

    /**
     * TargetSolution
     * 
     * @param errorCode Stores if the solver has encountered an error
     * @param launchSpeed Stores the launch speed of the projectile as an {@link LinearVelocity}
     * @param launchPitch Stores the launch pitch of the projectile as an {@link Angle}
     * @param launchYaw Stores the launch yaw of the projectile as an {@link Angle}
     * @param timestamp Stores the timestamp the result was calculated from as a {@link Time}
     */
    public record TargetSolution (
        TargetErrorCode errorCode,
        LinearVelocity launchSpeed,
        Angle launchPitch,
        Angle launchYaw,
        Time timestamp,
        
        TargetDebug targetDebug
    ) {};

    public record TargetInput (
        Angle robotYaw,
        Translation2d robotVelocity,
        Translation3d targetPosition,
        double efficiency,
        int maxSteps, 
        int tps
    ) {};

    /**
     * Calculates the launch pitch needed to reach a target not accounting for drag.
     * 
     * @param launchSpeed The launch speed of the projectile in meters per second
     * @param horizontalDistance The targets horizontal distance from the robot in meters
     * @param heightOffset The height offset of the target from the robot in meters
     * @return The pitch angle that the projectile should be launched in radians. Double.MAX_VALUE if the projectile cannot reach the target
     */
    private double calculateLaunchPitchIdeal(double launchSpeed, double horizontalDistance, double heightOffset) {
        if (Math.abs(horizontalDistance) < 1e-5) {
            return Double.MAX_VALUE; 
        }

        double discriminant = Math.pow(launchSpeed, 4) - (Math.pow(this.gravity, 2) * Math.pow(horizontalDistance, 2)) + (2 * this.gravity * Math.pow(launchSpeed, 2) * -heightOffset);

        if (discriminant < 0) {
            return Double.MAX_VALUE;
        }

        double square_root = Math.sqrt(discriminant);

        double numerator = (Math.pow(launchSpeed, 2)) + square_root;
        double denominator = this.gravity * horizontalDistance;

        return Math.atan(numerator / denominator);
    }

    /**
     * Simulates the launch of a projectile using runge kutta 4.
     * 
     * @param launchSpeed The launch velocity of the projectile in meters per second
     * @param launchPitch The launch pitch of the projectile in radians
     * @param launchYaw The launch yaw of the projectile in radians
     * @param launchAngularPitch The launch angular pitch of the projectile in radians per second
     * @param launchAngularYaw The launch angular yaw of the projectile in radians per second
     * @param robotVelocityX The field relative X velocity of the robot in meters per second
     * @param robotVelocityY The field relative Y velocity of the robot in meters per second
     * @param targetPositionX The relative X position of the target centered at the robot in meters
     * @param targetPositionY The relative Y position of the target centered at the robot in meters
     * @param targetPositionZ The relative Z position of the target centered at the robot in meters
     * @param robotYaw The yaw of the robot in radians
     * @param logAllPositions Controls if all positions of the ball during flight are returned
     * @param tps The ticks per second of the simulation
     * @return A {@link double} array in the format [Cur X, Cur Y, Cur Z, Prev X, Prev Y, Prev Z, Max Height]
     *  
     */
    public double[] simulateLaunch(
        double launchSpeed,
        double launchPitch,
        double launchYaw,
        double launchAngularPitch,
        double launchAngularYaw,
        double robotVelocityX,
        double robotVelocityY,
        double targetPositionX,
        double targetPositionY,
        double targetPositionZ,
        double robotYaw,
        boolean logAllPositions,
        int tps
    ) {
        
        double fuelVerticalOffset = Math.sin(launchPitch) * this.fuelPivotOffset;
        double fuelForwardOffset = Math.cos(launchPitch) * this.fuelPivotOffset;

        double fuelXOffset = Math.cos(launchYaw) * fuelForwardOffset;
        double fuelYOffset = Math.sin(launchYaw) * fuelForwardOffset;

        Translation3d rotatedTurretOffset = this.turretOffset.rotateBy(new Rotation3d(0, 0, robotYaw));

        double posX = rotatedTurretOffset.getX() + fuelXOffset;
        double posY = rotatedTurretOffset.getY() + fuelYOffset;
        double posZ = rotatedTurretOffset.getZ() + fuelVerticalOffset;

        double velX = (launchSpeed * Math.cos(launchPitch) * Math.cos(launchYaw)) + robotVelocityX;
        double velY = (launchSpeed * Math.cos(launchPitch) * Math.sin(launchYaw)) + robotVelocityY;
        double velZ = launchSpeed * Math.sin(launchPitch);

        double deltaTime = 1.0 / tps;

        double dragConstant = 0.5 * this.dragCoefficient * this.fluidDensity * this.crossSectionArea;
        double magnusConstant = 0.5 * this.liftCoefficient * this.fluidDensity * this.crossSectionArea * this.projectileRadius;
        double rotDragFactor = (0.5 * this.fluidDensity * this.rotDragCoefficient * this.crossSectionArea * Math.pow(this.projectileRadius, 3)) / this.momentOfInertia;

        double pitchSpinAxisYaw = launchYaw + (Math.PI / 2.0);

        double angX = launchAngularPitch * Math.cos(pitchSpinAxisYaw);
        double angY = launchAngularPitch * Math.sin(pitchSpinAxisYaw);
        double angZ = launchAngularYaw;

        double prevX = posX;
        double prevY = posY;
        double prevZ = posZ;

        double k1vx, k1vy, k1vz, k1ax, k1ay, k1az, k1wx, k1wy, k1wz, k1alphax, k1alphay, k1alphaz;
        double k2vx, k2vy, k2vz, k2ax, k2ay, k2az, k2wx, k2wy, k2wz, k2alphax, k2alphay, k2alphaz;
        double k3vx, k3vy, k3vz, k3ax, k3ay, k3az, k3wx, k3wy, k3wz, k3alphax, k3alphay, k3alphaz;
        double k4vx, k4vy, k4vz, k4ax, k4ay, k4az, k4wx, k4wy, k4wz, k4alphax, k4alphay, k4alphaz;

        double magVel, magAngVel, dragFactor;

        double magnusX, magnusY, magnusZ;

        double[] positionLog = logAllPositions ? new double[10 * tps * 3] : null;
        int logIndex = 0;
        double maxHeight = 0;

        for (int step = 0; step < 10 * tps; step++) {
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

            k1ax = (((k1vx * dragFactor) + magnusX) / this.mass);
            k1ay = (((k1vy * dragFactor) + magnusY) / this.mass);
            k1az = (((k1vz * dragFactor) + magnusZ) / this.mass) - this.gravity;
            
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

            k2ax = (((k2vx * dragFactor) + magnusX) / this.mass);
            k2ay = (((k2vy * dragFactor) + magnusY) / this.mass);
            k2az = (((k2vz * dragFactor) + magnusZ) / this.mass) - this.gravity;
            
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

            k3ax = (((k3vx * dragFactor) + magnusX) / this.mass);
            k3ay = (((k3vy * dragFactor) + magnusY) / this.mass);
            k3az = (((k3vz * dragFactor) + magnusZ) / this.mass) - this.gravity;
            
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

            k4ax = (((k4vx * dragFactor) + magnusX) / this.mass);
            k4ay = (((k4vy * dragFactor) + magnusY) / this.mass);
            k4az = (((k4vz * dragFactor) + magnusZ) / this.mass) - this.gravity;
            
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
            
            if (logAllPositions) {
                positionLog[logIndex++] = posX;
                positionLog[logIndex++] = posY;
                positionLog[logIndex++] = posZ;
            }

            if (posZ > maxHeight) {
                maxHeight = posZ;
            }

            if (velZ < 0 && posZ < targetPositionZ) {
                break;
            }
        }

        if (logAllPositions) {
            double[] result = new double[logIndex];
            System.arraycopy(positionLog, 0, result, 0, logIndex);
            return result;
        } else {
            return new double[] {
                posX, posY , posZ,
                prevX, prevY, prevZ,
                maxHeight
            };
        }
    }

    /**
     * Calculates the error of the projectiles landing point
     * 
     * @param launchSpeed The launch speed of the projectile in meters per second
     * @param launchPitch The launch pitch of the projectile in radians
     * @param launchYaw The launch yaw of the projectile in radians
     * @param launchAngularPitch The launch angular pitch of the projectile in radians per second
     * @param launchAngularYaw The launch angular yaw of the projectile in radians per second
     * @param robotVelocityX The field relative X velocity of the robot in meters per second
     * @param robotVelocityY The field relative Y velocity of the robot in meters per second
     * @param targetPositionX The relative X position of the target centered at the robot in meters
     * @param targetPositionY The relative Y position of the target centered at the robot in meters
     * @param targetPositionZ The relative Z position of the target centered at the robot in meters
     * @param robotYaw The yaw of the robot in radians
     * @param horizontalDistance The distance from the robot to the target in meters
     * @param tps The ticks per second of the simulation
     * @return The error in the simulation in the order:
     *  <ul>
     *       <li>Forward error</li>
     *       <li>Right error</li>
     *       <li>Vertical velocity</li>
     *       <li>Height error</li>
     *  </ul>
     */
    private double[] calculateLaunchError(
        double launchSpeed,
        double launchPitch,
        double launchYaw,
        double launchAngularPitch,
        double launchAngularYaw,
        double robotVelocityX,
        double robotVelocityY,
        double targetPositionX,
        double targetPositionY,
        double targetPositionZ,
        double robotYaw,
        double horizontalDistance,
        int tps
    ) {
        double[] path = simulateLaunch(
            launchSpeed,
            launchPitch,
            launchYaw,
            launchAngularPitch,
            launchAngularYaw,
            robotVelocityX,
            robotVelocityY,
            targetPositionX,
            targetPositionY,
            targetPositionZ,
            robotYaw,
            false,
            tps
        );

        double zHeight1 = path[5];
        double zHeight2 = path[2];

        double crossoverX, crossoverY, crossoverZ;
        if (Math.abs(zHeight2 - zHeight1) < 1e-9) {
            crossoverX = path[3];
            crossoverY = path[4];
            crossoverZ = path[5];
        } else {
            double weight = (targetPositionZ - zHeight1) / (zHeight2 - zHeight1);

            crossoverX = path[3] + ((path[0] - path[3]) * weight);
            crossoverY = path[4] + ((path[1] - path[4]) * weight);
            crossoverZ = path[5] + ((path[2] - path[5]) * weight);
        }

        double verticalVelocity = (path[2] - path[5]) * tps;
        
        double unitX = targetPositionX / horizontalDistance;
        double unitY = targetPositionY / horizontalDistance;
        
        double projectedDistanceForward = (crossoverX * unitX) + 
                                  (crossoverY * unitY);
        
        double projectedDistanceRight = (crossoverX * unitY) + 
                                  (crossoverY * -unitX);
        
        // Signed error: positive = overshot, negative = undershot
        double forwardError = projectedDistanceForward - horizontalDistance;

        double heightError = path[6] - (targetPositionZ + 2);

        if (path[5] < targetPositionZ) {
            
            forwardError = -5.0 - ((targetPositionZ - path[6]) * 3.0);
        }

        return new double[]{
            forwardError,
            projectedDistanceRight,
            verticalVelocity,
            heightError
        };
    }

    public LinearVelocity convertShooterSpeedToVelocity(AngularVelocity angularSpeed, Distance radius, double efficiency) {
        return MetersPerSecond.of((angularSpeed.in(RadiansPerSecond) * radius.in(Meter)) * efficiency);
    }

    public AngularVelocity convertVelocityToShooterSpeed(LinearVelocity linearVelocity, Distance radius, double efficiency) {
        return RadiansPerSecond.of((linearVelocity.in(MetersPerSecond) / radius.in(Meter)) / efficiency);
    }

    public LinearVelocity estimateShootingVelocity(Translation2d targetPosition, LinearVelocity speedLimitUpper, Translation2d robotVelocity, Angle robotYaw) {
        Translation3d rotatedTurretOffset = this.turretOffset.rotateBy(new Rotation3d(0, 0, robotYaw.in(Radians)));

        targetPosition = targetPosition.minus(rotatedTurretOffset.toTranslation2d());

        double targetDistance = targetPosition.getNorm();

        if (targetDistance < 1e-6) {
            return MetersPerSecond.of(speedLimitUpper.in(MetersPerSecond) * 0.4);
        }

        double baseVelocity = MathUtil.interpolate(
            speedLimitUpper.in(MetersPerSecond) * 0.18,
            speedLimitUpper.in(MetersPerSecond),
            MathUtil.inverseInterpolate(0, 32.5, targetDistance)
        );

        double normX = targetPosition.getX() / targetDistance;
        double normY = targetPosition.getY() / targetDistance;

        double radialVelocity = robotVelocity.getX() * normX + robotVelocity.getY() * normY;
        double tangentialVeloicty = robotVelocity.getX() * normY + robotVelocity.getY() * normX;

        double adjustedRadial = baseVelocity - (radialVelocity * ((radialVelocity > 0) ? 0.2 : 0.7));
        double adjustedTangential = tangentialVeloicty * 0.5;

        return MetersPerSecond.of(
            MathUtil.clamp(Math.sqrt(adjustedRadial * adjustedRadial + adjustedTangential * adjustedTangential), 0.0, speedLimitUpper.in(MetersPerSecond))
        );
    }

    public TargetSolution calculateLaunchAngleSimulation(TargetInput targetInput) {
        return calculateLaunchAngleSimulation(
            targetInput.robotYaw,
            targetInput.robotVelocity,
            targetInput.targetPosition,
            targetInput.efficiency,
            targetInput.maxSteps,
            targetInput.tps
        );
    }

    /**
     * 
     * @param robotYaw The yaw of the robot as a {@link Angle}
     * @param robotVelocity The field relative velocity of the robot as a {@link Translation2d} in Meters/Second
     * @param targetPosition The robot relative position of the target (Rotation is field relative) as a {@link Translation3d} in Meters
     * @param maxSteps The max amount of optimization steps (It can exit early if the error gets below a threshold)
     * @param tps The ticks per second that physics will be calculated at
     * @return The target solution
     */
    public TargetSolution calculateLaunchAngleSimulation(Angle robotYaw, Translation2d robotVelocity, Translation3d targetPosition, double efficiency, int maxSteps, int tps) {
        double startTime = Timer.getFPGATimestamp();

        double robotYawRadian = robotYaw.in(Radians);
        double robotVelocityX = robotVelocity.getX();
        double robotVelocityY = robotVelocity.getY();
        double targetPositionX = targetPosition.getX();
        double targetPositionY = targetPosition.getY();
        double targetPositionZ = targetPosition.getZ();
        
        double horizontalDistance = Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2));

        double speedLimitLower = convertShooterSpeedToVelocity(RadiansPerSecond.of(this.shooterMinVelocity), Meter.of(shooterWheelRadius), efficiency).in(MetersPerSecond);
        double speedLimitUpper = convertShooterSpeedToVelocity(RadiansPerSecond.of(this.shooterMaxVelocity), Meter.of(shooterWheelRadius), efficiency).in(MetersPerSecond);

        double theoreticalMaxSpeed = convertShooterSpeedToVelocity(RadiansPerSecond.of(this.shooterMaxVelocity), Meter.of(shooterWheelRadius), 1.0).in(MetersPerSecond);

        double launchSpeed = estimateShootingVelocity(targetPosition.toTranslation2d(), MetersPerSecond.of(theoreticalMaxSpeed), robotVelocity, robotYaw).in(MetersPerSecond);

        launchSpeed = MathUtil.clamp(launchSpeed, speedLimitLower, speedLimitUpper);


        double launchPitch = calculateLaunchPitchIdeal(speedLimitUpper, horizontalDistance, targetPositionZ - this.turretOffset.getZ());

        if (launchPitch == Double.MAX_VALUE) {
            return new TargetSolution(TargetErrorCode.IDEAL_PITCH, MetersPerSecond.of(0), Radians.of(0.0), Radians.of(0.0), Second.of(Timer.getTimestamp()), new TargetDebug(0, 0, 0, Second.of(Timer.getFPGATimestamp() - startTime)));
        }
        
        double launchYaw = Math.atan2(targetPositionY, targetPositionX);

        double perturbation = 0.05;

        double[] error = calculateLaunchError(launchSpeed, launchPitch, launchYaw, -(launchSpeed / this.projectileRadius), 0.0, robotVelocityX, robotVelocityY, targetPositionX, targetPositionY, targetPositionZ, robotYawRadian, horizontalDistance, tps);

        double[] pitchError = calculateLaunchError(launchSpeed, launchPitch + perturbation, launchYaw, -(launchSpeed / this.projectileRadius), 0.0, robotVelocityX, robotVelocityY, targetPositionX, targetPositionY, targetPositionZ, robotYawRadian, horizontalDistance, tps);

        double[] yawError = calculateLaunchError(launchSpeed, launchPitch, launchYaw + perturbation, -((launchSpeed) / this.projectileRadius), 0.0, robotVelocityX, robotVelocityY, targetPositionX, targetPositionY, targetPositionZ, robotYawRadian, horizontalDistance, tps);

        double pitchDelta;
        double yawDelta;

        double pitchForward = (pitchError[0] - error[0]) / perturbation;
        double pitchRight = (pitchError[1] - error[1]) / perturbation;
        double yawForward = (yawError[0] - error[0]) / perturbation;
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
                launchSpeed, 
                launchPitch, 
                launchYaw, 
                -(launchSpeed / this.projectileRadius), 
                0.0, 
                robotVelocityX,
                robotVelocityY,
                targetPositionX,
                targetPositionY,
                targetPositionZ,
                robotYawRadian,
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

            if (Math.abs(error[0]) < 0.03 && Math.abs(error[1]) < 0.03) {
                break;
            }
        }

        TargetErrorCode solutionFound = TargetErrorCode.NONE;
        if (Math.abs(launchYaw) > (Math.PI * 3)) {
            solutionFound = TargetErrorCode.EXCESSIVE_YAW;
        } else if (launchPitch > pitchLimitUpper) {
            solutionFound = TargetErrorCode.PITCH_UPPER_LIMIT;
        } else if (launchPitch < pitchLimitLower) {
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
            Second.of(Timer.getTimestamp()),
            new TargetDebug(pitchYawSteps, error[0], error[1], Second.of(Timer.getFPGATimestamp() - startTime))
        );

        
    }
}