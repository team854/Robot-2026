// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volt;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.ActivateIntakeCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.intake.ToggleIntakeDeployCommand;
import frc.robot.commands.kicker.ActivateKickerCommand;
import frc.robot.commands.kicker.ReverseKickerCommand;
import frc.robot.commands.smart.SmartShootCommand;
import frc.robot.commands.spindexer.ActivateSpindexerCommand;
import frc.robot.commands.spindexer.ReverseSpindexerCommand;
import frc.robot.commands.turret.ActivateShooterCommand;
import frc.robot.commands.turret.HomeTurretCommand;
import frc.robot.commands.turret.ManualStowTurretCommand;
import frc.robot.commands.turret.ResetTurretPitchCommand;
import frc.robot.commands.turret.ToggleFixedAimCommand;
import frc.robot.commands.turret.ToggleManualCommand;
import frc.robot.commands.turret.TurretAutoAimCommand;
import frc.robot.libraries.FieldHelpers;
import frc.robot.libraries.ProjectileSimulation.TargetErrorCode;
import frc.robot.libraries.control.ControllerIO;
import frc.robot.libraries.control.ControllerIOPS5;
import frc.robot.libraries.control.ControllerIOXbox;
import frc.robot.subsystems.drive.SwerveIO;
import frc.robot.subsystems.drive.SwerveIOReal;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeDeploymentIO;
import frc.robot.subsystems.intake.IntakeDeploymentIOReal;
import frc.robot.subsystems.intake.IntakeDeploymentSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightSubsystem;
import frc.robot.subsystems.logging.HealthSubsystem;
import frc.robot.subsystems.logging.VisualizerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOReal;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.CalculationSubsystem;
import frc.robot.subsystems.turret.KickerIO;
import frc.robot.subsystems.turret.KickerIOReal;
import frc.robot.subsystems.turret.KickerSubsystem;
import frc.robot.subsystems.turret.ShooterIO;
import frc.robot.subsystems.turret.ShooterIOKrakenReal;
import frc.robot.subsystems.turret.ShooterIOReal;
import frc.robot.subsystems.turret.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOKrakenReal;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
	public static final ControllerIO driverController = Robot.isReal() ? new ControllerIOXbox(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT) : new ControllerIOPS5(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

	// Establishes subsystems
	public static final HealthSubsystem healthSubsystem = new HealthSubsystem();
	public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
		Constants.SwerveConstants.ENABLED ? new SwerveIOReal() : new SwerveIO() {}
	);
	public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(
		Constants.ShooterConstants.ENABLED ? new ShooterIOKrakenReal() : new ShooterIO() {}
	);
	public static final TurretSubsystem turretSubsystem = new TurretSubsystem(
		Constants.TurretConstants.ENABLED ? new TurretIOKrakenReal() : new TurretIO() {}
	);
	public static final KickerSubsystem kickerSubsystem = new KickerSubsystem(
		Constants.KickerConstants.ENABLED ? new KickerIOReal() : new KickerIO() {}
	);
	public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
		Constants.IntakeConstants.ENABLED ? new IntakeIOReal() : new IntakeIO() {}
	);
	public static final IntakeDeploymentSubsystem intakeDeploymentSubsystem = new IntakeDeploymentSubsystem(
		Constants.IntakeConstants.ENABLED ? new IntakeDeploymentIOReal() : new IntakeDeploymentIO() {}
	);
	public static final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem(
		Constants.SpindexerConstants.ENABLED ? new SpindexerIOReal() : new SpindexerIO() {}
	);

	public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
	public static final QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();
	public static final VisualizerSubsystem visualizerSubsystem = new VisualizerSubsystem();
	public static final LightSubsystem lightSubsystem = new LightSubsystem();
	public static final CalculationSubsystem calculationSubsystem = new CalculationSubsystem();
	
	private static SendableChooser<Command> autoChooser;

	// Transforms controller input into swerve drive spee
	public static SwerveInputStream swerveInputStream;
	public static Supplier<ChassisSpeeds> swerveChassisSpeedsSupplier = () -> {return new ChassisSpeeds();};

	public static boolean rotationalAiming = false;
	private boolean turretHomed = false;
	private Command turretHomeCommand;

	public static Command driveFieldOrientedAngularVelocity;
	
	private int lastTeam = -1;

	private LinearFilter batteryVoltageFilter = LinearFilter.movingAverage(50);

	public RobotContainer() {
		if (Constants.SwerveConstants.ENABLED) {
			swerveInputStream = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
				driverController.leftXCombinedSupplier(),//() -> driverController.getLeftY() * -1,
				driverController.leftYCombinedSupplier())//() -> driverController.getLeftX() * -1)
				.withControllerRotationAxis(
					driverController.rightXSupplier()
				)
				.withControllerHeadingAxis(
					calculationSubsystem.getTargetHeadingX(),
					calculationSubsystem.getTargetHeadingY()
				)
				.deadband(0.0001)
				.scaleRotation(Constants.OperatorConstants.SWERVE_ROTATION_SCALE)
				.scaleTranslation(Constants.OperatorConstants.SWERVE_TRANSLATION_SCALE)
				.headingWhile(() -> {return RobotContainer.rotationalAiming;})
				.allianceRelativeControl(true);

			swerveChassisSpeedsSupplier = () -> {return swerveInputStream.get();};
			driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(swerveChassisSpeedsSupplier);

			
		} else {
			driveFieldOrientedAngularVelocity = swerveSubsystem.run(() -> {});
		}

		registerCommands();

		configureBindings();

		if (Constants.SwerveConstants.ENABLED) {
			autoChooser = AutoBuilder.buildAutoChooser();
			autoChooser.addOption("Right Double Swipe", new PathPlannerAuto("Left Double Swipe", true));
		} else {
			autoChooser = new SendableChooser<Command>();
		}

		autoChooser.addOption("Home Turret Command", new HomeTurretCommand());
		autoChooser.addOption("Shooter Flywheel SysID Command", shooterSubsystem.getFlywheelSysIDCommand());

		SmartDashboard.putData("Auto Chooser", autoChooser);


		//DO SANITY CHECKS OF THE MAGNUS EFFECT
		/*
		if (false) {
			TargetSolution solution = projectileSimulation.calculateLaunchAngleSimulation(
				projectileSimulation.convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5),
				DegreesPerSecond.of(0),
				new Translation2d(0, 2),
				new Translation3d(4.4,0,1.9558),
				Constants.FuelPhysicsConstants.MAX_STEPS,
				Constants.FuelPhysicsConstants.TPS
				
			);
			System.out.println(10.09999999999998 + " " + solution.toString());
		} else {
			for (double px = 2; px < 14; px+=0.1) {
				TargetSolution solution = projectileSimulation.calculateLaunchAngleSimulation(
					projectileSimulation.convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5),
					DegreesPerSecond.of(0),
					new Translation2d(0, 0),
					new Translation3d(px,0,1.9558),
					Constants.FuelPhysicsConstants.MAX_STEPS,
					Constants.FuelPhysicsConstants.TPS
					
				);
				System.out.println(px + " " + solution.toString());
				
			}
			System.out.println("FEWF");
			
		}
		*/
		
	}

	private void registerCommands() {
		NamedCommands.registerCommand("RetractIntakeCommand", new RetractIntakeCommand());
		NamedCommands.registerCommand("DeployIntakeCommand", new DeployIntakeCommand());

		NamedCommands.registerCommand("HomeTurretCommand", new HomeTurretCommand());
		NamedCommands.registerCommand("TurretAutoAimCommand", new TurretAutoAimCommand());

		NamedCommands.registerCommand("ActivateSpindexerCommand", new ActivateSpindexerCommand());
		NamedCommands.registerCommand("ActivateKickerCommand", new ActivateKickerCommand());
		NamedCommands.registerCommand("ActivateShooterCommand", new ActivateShooterCommand());
		NamedCommands.registerCommand("ActivateIntakeCommand", new ActivateIntakeCommand());

		NamedCommands.registerCommand("SmartShootCommand", new SmartShootCommand());
	}

	private void configureBindings() {
		if (Constants.SwerveConstants.ENABLED) {
			swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
		}

		turretSubsystem.setDefaultCommand(new TurretAutoAimCommand());

		driverController.leftStick().toggleOnTrue(new ToggleManualCommand(
			() -> {return 0.0;}, driverController.rightYSupplier()
		));

		driverController.rightStick().toggleOnTrue(
			new InstantCommand(() -> {
				if (rotationalAiming) {
					rotationalAiming = false;
				} else {
					rotationalAiming = true;
				}
			})
		);

		driverController.rightButton().toggleOnTrue(new ActivateShooterCommand());

		driverController.menuButton().onTrue(RobotContainer.swerveSubsystem.zeroGyroCommand());

		driverController.leftBumper().whileTrue(new ManualStowTurretCommand());

		driverController.backButton().onTrue(new ResetTurretPitchCommand());
		
		driverController.rightTrigger().whileTrue(new ParallelCommandGroup(
				new ActivateKickerCommand(),
				new ActivateSpindexerCommand()
			)
		);
		
		driverController.rightBumper().whileTrue(new ParallelCommandGroup(
				new ReverseKickerCommand(),
				new ReverseSpindexerCommand()
			)
		);

		driverController.upButton().toggleOnTrue(new ToggleFixedAimCommand());
		
		//driverController.rightTrigger().whileTrue(new SmartShootCommand());

		driverController.leftButton().toggleOnTrue(new ToggleIntakeDeployCommand());

		driverController.leftTrigger().whileTrue(new ActivateIntakeCommand());
	}

	public static boolean isBlueAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() != DriverStation.Alliance.Red : true;
    }

	public static boolean isRedAlliance() {
		return !isBlueAlliance();
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void periodicHealthChecks() {
		

		if (!DriverStation.isDSAttached()) {
			healthSubsystem.reportError("Main", ErrorConstants.DS_DISCONNECTED);
			healthSubsystem.clearError("Main", ErrorConstants.JOYSTICKS_DISCONNECTED);
		} else {
			if (!DriverStation.isJoystickConnected(0)) {
				healthSubsystem.reportError("Main", ErrorConstants.JOYSTICKS_DISCONNECTED);
			} else {
				healthSubsystem.clearError("Main", ErrorConstants.JOYSTICKS_DISCONNECTED);
			}

			healthSubsystem.clearError("Main", ErrorConstants.DS_DISCONNECTED);
		}

		double batteryVoltage = RobotController.getBatteryVoltage();
		double smoothedVoltage = batteryVoltageFilter.calculate(batteryVoltage);
		if (smoothedVoltage < Constants.HealthConstants.LOW_BATTERY_THRESHOLD.in(Volt)) {
			healthSubsystem.reportError("Main", ErrorConstants.LOW_BATTERY_VOLTAGE);
		} else {
			healthSubsystem.clearError("Main", ErrorConstants.LOW_BATTERY_VOLTAGE);
		}
	}

	public void initAll() {

		shooterSubsystem.resetShooter();

		calculationSubsystem.updateAimingPositions();

		int team = (RobotContainer.isBlueAlliance() ? 1 : 0);
		if (lastTeam != team) {
			swerveSubsystem.resetOdometry(
				FieldHelpers.rotateBlueFieldCoordinates(new Translation2d(Meter.of(2), Meter.of(4)), isRedAlliance())
			);
			lastTeam = team;
		}

		if (!turretHomed) {
			/*
			if (Robot.isReal()) {
				CommandScheduler.getInstance().cancel(turretHomeCommand);

				turretHomeCommand = new SequentialCommandGroup(new HomeTurretCommand(), Commands.runOnce(() -> {turretHomed = true;}));
				CommandScheduler.getInstance().schedule(turretHomeCommand);
			} else {
				turretHomed = true;
			}
			*/
		}
		
	}

	public void periodic() {
		
		Pose2d botPose = swerveSubsystem.getPose2d();

		calculationSubsystem.updateBotZone(botPose);
		calculationSubsystem.updateTrajectoryCalculations(botPose);
		
	}
}
