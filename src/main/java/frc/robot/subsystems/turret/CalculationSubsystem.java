package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Second;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

	private final ProjectileSimulation projectileInstance = new ProjectileSimulation();

    private Thread projectileThread;

    public CalculationSubsystem() {
        targetInputs = new AtomicReference<>(new TargetInput(MetersPerSecond.of(1), DegreesPerSecond.of(0), new Translation2d(), new Translation3d(), 1, 1));
        targetSolutions = new AtomicReference<>(new TargetSolution(TargetErrorCode.IDEAL_PITCH, MetersPerSecond.of(10), Degree.of(0), Degree.of(0), Second.of(0), new TargetDebug(0, 0, 0)));
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
            blueHub.toTranslation2d().plus(new Translation2d(0, Constants.FieldConstants.TRENCH_OFFSET.in(Meter))),
            blueHub.toTranslation2d().plus(new Translation2d(0, -Constants.FieldConstants.TRENCH_OFFSET.in(Meter))),
            redHub.toTranslation2d().plus(new Translation2d(0, Constants.FieldConstants.TRENCH_OFFSET.in(Meter))),
            redHub.toTranslation2d().plus(new Translation2d(0, -Constants.FieldConstants.TRENCH_OFFSET.in(Meter)))
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

        for (int index = 0; index < 4; index++) {
            double distance = botPose.getTranslation().getDistance(trenchZone[index]);
            if (distance < Constants.FieldConstants.TRENCH_RADIUS.in(Meter)) {
                botZone = Zone.TRENCH;
                return;
            }
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

        Translation3d robotTargetRelative = new Translation3d(
            targetPosition.getX() - botPose.getX(),
            targetPosition.getY() - botPose.getY(),
            targetPosition.getZ()
        );

        ChassisSpeeds fieldSpeeds = RobotContainer.swerveSubsystem.getFieldChassisSpeeds();

        setTargetInputs(new TargetInput(
            getProjectileSimulation().convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5),
            DegreesPerSecond.of(0),
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond
            ),
            robotTargetRelative,
            Constants.FuelPhysicsConstants.MAX_STEPS,
            Constants.FuelPhysicsConstants.TPS
        ));

        TargetSolution lastSolution = getTargetSolutions();

        SmartDashboard.putNumberArray("Auto Aim/Target Position", PoseHelpers.convertTranslationToNumbers(targetPosition));
        SmartDashboard.putString("Auto Aim/Error Code", lastSolution.errorCode().name());
        SmartDashboard.putString("Auto Aim/Solution Debug", lastSolution.targetDebug().toString());
        SmartDashboard.putBoolean("Auto Aim/Error", lastSolution.errorCode() != TargetErrorCode.NONE);
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
                double startTime = Timer.getFPGATimestamp();

                TargetInput targetInput = this.getTargetInputs();

                TargetSolution solution = projectileSimulationInstance.calculateLaunchAngleSimulation(targetInput);
                this.setTargetSolutions(solution);

                double elapsedTime = Timer.getFPGATimestamp() - startTime;
                long sleepTimeMs = Math.max(5, 20 - (long)(elapsedTime * 1000));
                
                try {
                    Thread.sleep(sleepTimeMs);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
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
