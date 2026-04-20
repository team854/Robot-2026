package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Second;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.FieldHelpers;
import frc.robot.libraries.PoseHelpers;
import frc.robot.libraries.ProjectileSimulation;
import frc.robot.libraries.ProjectileSimulation.TargetDebug;
import frc.robot.libraries.ProjectileSimulation.TargetErrorCode;
import frc.robot.libraries.ProjectileSimulation.TargetInput;
import frc.robot.libraries.ProjectileSimulation.TargetSolution;

public class CalculationSubsystem {
    public enum Zone {
        ALLIANCE,
        OTHER_HIGH_CENTER,
        OTHER_LOW_CENTER,
        TRENCH
    }

    private Zone botZone = Zone.ALLIANCE;

    private Translation3d hubPosition = new Translation3d();
    private Translation3d[] passPosition = new Translation3d[] {new Translation3d(), new Translation3d()};

    private Translation2d[] allianceZone = new Translation2d[] {new Translation2d(), new Translation2d()};

    private Translation2d[] trenchZone = new Translation2d[] {new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()};

    private Translation3d targetPosition = new Translation3d();

    private final AtomicReference<TargetInput> targetInputs;
    private final AtomicReference<TargetSolution> targetSolutions;

    private double previousVelocityX = 0;
    private double previousVelocityY = 0;
    private double previousTimestamp = 0;

    private final LinearFilter velocityFilterX = LinearFilter.movingAverage(5);
    private final LinearFilter velocityFilterY = LinearFilter.movingAverage(5);

	private final ProjectileSimulation projectileInstance = new ProjectileSimulation(
        Constants.FuelPhysicsConstants.DRAG_CONSTANT,
        Constants.FuelPhysicsConstants.ROT_DRAG_CONSTANT,
        Constants.FuelPhysicsConstants.CROSS_SECTION_AREA,
        Constants.FuelPhysicsConstants.MASS,
        Constants.FuelPhysicsConstants.FLUID_DENSITY,
        Constants.FuelPhysicsConstants.GRAVITY,
        Constants.FuelPhysicsConstants.LIFT_CONSTANT,
        Constants.TurretConstants.TURRET_PIVOT_FUEL_OFFSET,
        Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS,
        Constants.TurretConstants.TURRET_PIVOT_OFFSET,
        Radian.of((Math.PI / 2.0) - Constants.TurretConstants.TURRET_PITCH_UPPER_LIMIT.in(Radian)),
        Radian.of((Math.PI / 2.0) - Constants.TurretConstants.TURRET_PITCH_LOWER_LIMIT.in(Radian)),
        Constants.ShooterConstants.SHOOTER_MIN_VELOCITY,
        Constants.ShooterConstants.SHOOTER_MAX_VELOCITY
    );

    private Thread projectileThread;

    private final LinearFilter simulationTimeFilter = LinearFilter.movingAverage(500);

    public CalculationSubsystem() {
        targetInputs = new AtomicReference<>(new TargetInput(Radian.of(0), new Translation2d(), new Translation3d(), 0.5, 1, 1));
        targetSolutions = new AtomicReference<>(new TargetSolution(TargetErrorCode.IDEAL_PITCH, MetersPerSecond.of(10), Degree.of(0), Degree.of(0), Second.of(0), new TargetDebug(0, 0, 0, Second.of(0))));
    }

    public ProjectileSimulation getProjectileSimulation() {
        return projectileInstance;
    }

    public void updateAimingPositions() {
        Translation3d blueHub = new Translation3d(
                Constants.FieldConstants.HUB_SIDE_DISTANCE.in(Meter),
                Constants.FieldConstants.FIELD_SIZE_Y.in(Meter) / 2.0,
                Constants.FieldConstants.HUB_TARGET_HEIGHT.in(Meter)
            );
        Translation3d redHub = FieldHelpers.rotateBlueFieldCoordinates(blueHub,true);

        hubPosition = FieldHelpers.rotateBlueFieldCoordinates(
            blueHub,
            !RobotContainer.isBlueAlliance()
        );

        Translation3d rotatedPassPosition = FieldHelpers.rotateBlueFieldCoordinates(
            new Translation3d(
                Constants.FieldConstants.PASS_SIDE_DISTANCE.in(Meter),
                Constants.FieldConstants.FIELD_SIZE_Y.in(Meter) / 2.0,
                0
            ),
            !RobotContainer.isBlueAlliance()
        );

        passPosition = new Translation3d[]{
            rotatedPassPosition.plus(new Translation3d(0, Constants.FieldConstants.PASS_OFFSET.in(Meter), 0)),
            rotatedPassPosition.plus(new Translation3d(0, -Constants.FieldConstants.PASS_OFFSET.in(Meter), 0))
        };

        trenchZone = new Translation2d[] {
            blueHub.toTranslation2d().plus(new Translation2d(-1, -20)),
            blueHub.toTranslation2d().plus(new Translation2d(1, 20)),
            redHub.toTranslation2d().plus(new Translation2d(-1,-20)),
            redHub.toTranslation2d().plus(new Translation2d(1,20))
        };

        if (RobotContainer.isBlueAlliance()) {
            allianceZone = new Translation2d[] {
                new Translation2d(0,0),
                new Translation2d(Constants.FieldConstants.HUB_SIDE_DISTANCE.in(Meter), Constants.FieldConstants.FIELD_SIZE_Y.in(Meter))
            };
        } else {
            allianceZone = new Translation2d[] {
                new Translation2d(Constants.FieldConstants.FIELD_SIZE_X.in(Meter) - Constants.FieldConstants.HUB_SIDE_DISTANCE.in(Meter),0),
                new Translation2d(Constants.FieldConstants.FIELD_SIZE_X.in(Meter), Constants.FieldConstants.FIELD_SIZE_Y.in(Meter))
            };
        }
    }

    public void updateBotZone(Pose2d botPose) {

        if ((trenchZone[0].getX() <= botPose.getX() && 
            trenchZone[0].getY() <= botPose.getY() &&
            trenchZone[1].getX() >= botPose.getX() && 
            trenchZone[1].getY() >= botPose.getY()) ||
            (trenchZone[2].getX() <= botPose.getX() && 
            trenchZone[2].getY() <= botPose.getY() &&
            trenchZone[3].getX() >= botPose.getX() && 
            trenchZone[3].getY() >= botPose.getY())) {
            botZone = Zone.TRENCH;
            return;
        }

        
        if (allianceZone[0].getX() <= botPose.getX() && allianceZone[0].getY() <= botPose.getY() &&
            allianceZone[1].getX() >= botPose.getX() && allianceZone[1].getY() >= botPose.getY()) {
            botZone = Zone.ALLIANCE;
        } else {
            if (botPose.getY() > (Constants.FieldConstants.FIELD_SIZE_Y.in(Meter) / 2.0)) {
                botZone = Zone.OTHER_HIGH_CENTER;
            } else {
                botZone = Zone.OTHER_LOW_CENTER;
            }
        }
    }

    public void updateTrajectoryCalculations(Pose2d botPose) {
        
        switch (botZone) {
            case ALLIANCE:
                targetPosition = hubPosition;
                break;
            case OTHER_HIGH_CENTER:
                targetPosition = passPosition[0];
                break;
            case OTHER_LOW_CENTER:
                targetPosition = passPosition[1];
                break;
            case TRENCH:
                targetPosition = hubPosition;
                break;
        }

        double timestamp = Timer.getFPGATimestamp();
        ChassisSpeeds fieldSpeeds = RobotContainer.swerveSubsystem.getFieldChassisSpeeds();
        ChassisSpeeds targetSpeeds = RobotContainer.swerveChassisSpeedsSupplier.get();

        double deltaTime = MathUtil.clamp(timestamp - previousTimestamp, 0.005, 0.1);

        double velocityX = velocityFilterX.calculate(fieldSpeeds.vxMetersPerSecond);
        double velocityY = velocityFilterY.calculate(fieldSpeeds.vyMetersPerSecond);

        double accelerationX = (velocityX - previousVelocityX) / deltaTime;
        double accelerationY = (velocityY - previousVelocityY) / deltaTime;

        previousVelocityX = velocityX;
        previousVelocityY = velocityY;
        previousTimestamp = timestamp;

        double predictedVelocityX = velocityX + (accelerationX * Constants.TurretConstants.TURRET_LATENCY.in(Second));
        double predictedVelocityY = velocityY + (accelerationY * Constants.TurretConstants.TURRET_LATENCY.in(Second));

        predictedVelocityX = MathUtil.clamp(
            predictedVelocityX,
            -Math.max(Math.abs(velocityX), Math.abs(targetSpeeds.vxMetersPerSecond)),
            Math.max(Math.abs(velocityX), Math.abs(targetSpeeds.vxMetersPerSecond))
        );
        predictedVelocityY = MathUtil.clamp(
            predictedVelocityY,
            -Math.max(Math.abs(velocityY), Math.abs(targetSpeeds.vyMetersPerSecond)),
            Math.max(Math.abs(velocityY), Math.abs(targetSpeeds.vyMetersPerSecond))
        );

        double predictedAccelerationX = (predictedVelocityX - velocityX) / Constants.TurretConstants.TURRET_LATENCY.in(Second);
        double predictedAccelerationY = (predictedVelocityY - velocityY) / Constants.TurretConstants.TURRET_LATENCY.in(Second);

        double predictedPositionX = botPose.getX() + ((velocityX * Constants.TurretConstants.TURRET_LATENCY.in(Second)) + (0.5 * predictedAccelerationX * Math.pow(Constants.TurretConstants.TURRET_LATENCY.in(Second), 2)));
        double predictedPositionY = botPose.getY() + ((velocityY * Constants.TurretConstants.TURRET_LATENCY.in(Second)) + (0.5 * predictedAccelerationY * Math.pow(Constants.TurretConstants.TURRET_LATENCY.in(Second), 2)));

        Translation3d robotTargetRelative = new Translation3d(
            targetPosition.getX() - predictedPositionX,
            targetPosition.getY() - predictedPositionY,
            targetPosition.getZ()
        );

        setTargetInputs(new TargetInput(
            botPose.getRotation().getMeasure(),
            new Translation2d(
                predictedVelocityX,
                predictedVelocityY
            ),
            robotTargetRelative,
            Constants.FuelPhysicsConstants.EFFICENCY,
            Constants.FuelPhysicsConstants.MAX_STEPS,
            Constants.FuelPhysicsConstants.TPS
        ));

        TargetSolution lastSolution = getTargetSolutions();
        double filteredComputationTime = simulationTimeFilter.calculate(lastSolution.targetDebug().computationTime().in(Second));

        SmartDashboard.putNumberArray("Auto Aim/Target Position", PoseHelpers.convertTranslationToNumbers(targetPosition));
        SmartDashboard.putString("Auto Aim/Error Code", lastSolution.errorCode().name());
        SmartDashboard.putString("Auto Aim/Solution Debug", lastSolution.targetDebug().toString());
        SmartDashboard.putNumber("Auto Aim/Timestamp", lastSolution.timestamp().in(Second));
        SmartDashboard.putBoolean("Auto Aim/Error", lastSolution.errorCode() != TargetErrorCode.NONE);
        SmartDashboard.putNumber("Auto Aim/Smoothed Computation Time", filteredComputationTime);

        SmartDashboard.putNumberArray("Auto Aim/Predicted Position", PoseHelpers.convertTranslationToNumbers(new Translation3d(predictedPositionX, predictedPositionY, 0.0)));

        SmartDashboard.putNumber("Auto Aim/Predicted Velocity X", predictedVelocityX);
        SmartDashboard.putNumber("Auto Aim/Predicted Velocity Y", predictedVelocityY);

        SmartDashboard.putNumber("Auto Aim/Predicted Acceleration X", predictedAccelerationX);
        SmartDashboard.putNumber("Auto Aim/Predicted Acceleration Y", predictedAccelerationY);
    }

    public void setTargetInputs(TargetInput targetInput) {
        targetInputs.set(targetInput);
    }

    public TargetInput getTargetInputs() {
        return targetInputs.get();
    }

    public void setTargetSolutions(TargetSolution targetSolution) {
        targetSolutions.set(targetSolution);
    }

    public TargetSolution getTargetSolutions() {
        return targetSolutions.get();
    }

    public void startPhysicsSimulation() {
        if (projectileThread != null && projectileThread.isAlive()) {
            return; 
        }

        projectileThread = new Thread(() -> {

            ProjectileSimulation projectileSimulationInstance = this.getProjectileSimulation();

            while (!Thread.currentThread().isInterrupted()) {
                try {
                    double startTime = Timer.getFPGATimestamp();

                    TargetInput targetInput = this.getTargetInputs();

                    TargetSolution solution = projectileSimulationInstance.calculateLaunchAngleSimulation(targetInput);
                    this.setTargetSolutions(solution);

                    double elapsedTime = Timer.getFPGATimestamp() - startTime;

                    long sleepTimeMs = Math.max(5, 20 - (long)(elapsedTime * 1000));

                    Thread.sleep(sleepTimeMs);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    DriverStation.reportError("Projectile Simulation Thread Crashed: " + e.getMessage(), e.getStackTrace());
                }
            }
        });
        projectileThread.setDaemon(true);
        projectileThread.setName("Projectile Simulation Thread");
        projectileThread.start();
    }

    public Zone getZone() {
        return botZone;
    }

    public Translation3d getTargetPosition() {
        return targetPosition;
    }

    public DoubleSupplier getTargetHeadingX() {
        return () -> {
            Translation2d offset = getTargetPosition().toTranslation2d().minus(RobotContainer.swerveSubsystem.getPose2d().getTranslation());
            double angle = Math.atan2(-offset.getY(), -offset.getX());
            return Math.sin(angle);
        };
    }

    public DoubleSupplier getTargetHeadingY() {
        return () -> {
            Translation2d offset = getTargetPosition().toTranslation2d().minus(RobotContainer.swerveSubsystem.getPose2d().getTranslation());
            double angle = Math.atan2(-offset.getY(), -offset.getX());
            return Math.cos(angle);
        };
    }
}
